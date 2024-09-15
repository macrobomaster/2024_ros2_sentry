from enum import Enum
import time
import serial
import struct
import re
import math
import numpy as np

class CvCmdHandler:
    # misc constants
    DATA_TIMESTAMP_INDEX = 2
    DATA_PACKAGE_SIZE = 21  # 2 bytes header, 1 byte msg type, 16 bytes payload
    DATA_PAYLOAD_INDEX = 5
    MIN_TX_SEPARATION_SEC = 0  # reserved for future, currently control board is fast enough
    MIN_INFO_REQ_SEPARATION_SEC = 0
    # Note: for better performance, shoot timeout in cv board must be smaller than the timeout in control board
    SHOOT_TIMEOUT_SEC = 0
    DEBUG_CV = False

    class eMsgType(Enum):
        MSG_MODE_CONTROL = b'\x10'
        MSG_CV_CMD = b'\x20'
        MSG_ACK = b'\x40'
        MSG_INFO_REQUEST = b'\x50'
        MSG_INFO_FEEDBACK = b'\x51'

    class eSepChar(Enum):  # start and ending hexes, acknowledgement bit
        CHAR_HEADER = b'>>'
        ACK_ASCII = b'ACK'
        CHAR_UNUSED = b'\xFF'

    class eRxState(Enum):
        RX_STATE_INIT = 0
        RX_STATE_WAIT_FOR_PKG = 1
        RX_STATE_SEND_ACK = 2

    class eModeControlBits(Enum):
        MODE_AUTO_AIM_BIT = 0b00000001
        MODE_AUTO_MOVE_BIT = 0b00000010
        MODE_ENEMY_DETECTED_BIT = 0b00000100
        MODE_SHOOT_BIT = 0b00001000
        MODE_CHASSIS_SPINNING_BIT = 0b00010000

    class eInfoBits(Enum):
        MODE_TRAN_DELTA_BIT = 0b00000001
        MODE_CV_SYNC_TIME_BIT = 0b00000010
        MODE_REF_STATUS_BIT = 0b00000100
        MODE_GIMBAL_ANGLE_BIT = 0b00001000

    class eTeamColor(Enum):
        TEAM_COLOR_BLUE = 0
        TEAM_COLOR_RED = 1

    def __init__(self, serial_port):
        self.ackMsgInfo = {"reqCtrlTimestamp": -1, "reqRxTimestamp": -1}
        self.CvSyncTime = 0
        self.CvCmd_Reset()

        # Example port on ubuntu: port='/dev/ttyTHS2'
        self.ser = serial.Serial(port=serial_port, baudrate=1000000, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

        self.txCvCmdMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_CV_CMD.value)
        self.txInfoRequestMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_INFO_REQUEST.value)
        self.txSetModeMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_MODE_CONTROL.value)
        self.txAckMsg = self.CvCmd_InitTxMsg(self.eMsgType.MSG_ACK.value + self.eSepChar.ACK_ASCII.value)

        # @TODO: reserve data placeholders within msg frames for each type of msg beforehand
        self.txAckMsgPayloadIndex = self.DATA_PAYLOAD_INDEX + len(self.eSepChar.ACK_ASCII.value)

    def CvCmd_InitTxMsg(self, payload):
        txMsg = bytearray(self.eSepChar.CHAR_HEADER.value)
        txMsg += b'\x00\x00'  # placeholder for timestamp
        txMsg += payload
        txMsg += self.eSepChar.CHAR_UNUSED.value*(self.DATA_PACKAGE_SIZE-len(txMsg))
        return txMsg

    def CvCmd_GetUint16Time(self):
        return int(time.time()*1000) & ((1 << 16)-1)

    def CvCmd_GetUint16Delta(self, minuendTime, subtrahendTime):
        signedDelta = minuendTime - subtrahendTime
        unsignedDelta = signedDelta if (signedDelta >= 0) else (signedDelta + (1 << 16))
        return unsignedDelta

    def CvCmd_BuildCvControlledTxSetModeMsg(self):
        self.txSetModeMsg[self.DATA_PAYLOAD_INDEX] = 0
        if self.ShootSwitch:
            self.txSetModeMsg[self.DATA_PAYLOAD_INDEX] |= self.eModeControlBits.MODE_SHOOT_BIT.value

        if self.ChassisSpinningSwitch:
            self.txSetModeMsg[self.DATA_PAYLOAD_INDEX] |= self.eModeControlBits.MODE_CHASSIS_SPINNING_BIT.value

    def CvCmd_BuildSendTxMsg(self, txMsg):
        txMsg[self.DATA_TIMESTAMP_INDEX:self.DATA_TIMESTAMP_INDEX+2] = struct.pack('<H', self.CvCmd_GetUint16Delta(self.CvCmd_GetUint16Time(), self.CvSyncTime))
        self.ser.write(txMsg)
        self.prevTxTime = time.time()
        if self.DEBUG_CV:
            assert (len(txMsg) == self.DATA_PACKAGE_SIZE)
            print("Sent: ", txMsg)
            if txMsg is self.txAckMsg:
                print("ACK sent")
            elif txMsg is self.txInfoRequestMsg:
                print("Info request sent")
            elif txMsg is self.txCvCmdMsg:
                print("Gimbal cmd sent")
            elif txMsg is self.txSetModeMsg:
                print("Shoot req sent")

    def CvCmd_Reset(self):
        # reqCtrlTimestamp: control native timestamp contained within the request sent by control board
        # reqRxTimestamp: receive timestamp of the request sent by control board
        self.ackMsgInfo["reqCtrlTimestamp"] = -1
        self.ackMsgInfo["reqRxTimestamp"] = -1
        self.infoRequestPending = 0
        self.prevTxTime = 0
        self.prevInfoReqTime = 0
        self.refStatusUpdateTime = 0
        self.gimbalStatusUpdateTime = 0
        # self.cvCmdCount = 0
        self.tranDelta = None  # Transmission delay time in ms
        self.AutoAimSwitch = False
        self.AutoMoveSwitch = False
        self.EnemySwitch = False
        self.ShootSwitch = False
        self.ChassisSpinningSwitch = False
        self.prevChassisSpinningSwitch = False
        self.chassis_cmd_speed_x = 0
        self.chassis_cmd_speed_y = 0
        self.chassis_speed_x = 0
        self.chassis_speed_y = 0
        self.Rx_State = self.eRxState.RX_STATE_INIT
        self.prev_Rx_State = self.Rx_State
        self.time_remain = 0
        self.current_HP = 0
        self.game_progress = 0
        self.teamColor = self.eTeamColor.TEAM_COLOR_BLUE.value
        self.shootStartTime = 0
        self.gimbal_yaw_angle = 0
        self.gimbal_pitch_angle = 0
        self.RedOutpostHP = 0
        self.BlueOutpostHP = 0
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except:
            pass

    # @param[out]: (type uint8_t) 1 means game on, 0 means game off
    def CvCmd_GetGameProgress(self):
        return self.game_progress

    # @param[out]: (type uint16_t) time remaining in current game section
    def CvCmd_GetGameTimeRemain(self):
        return self.time_remain

    # @param[out]: (type uint16_t) health points remaining
    def CvCmd_GetCurrentHp(self):
        return self.current_HP

    # @param[out]: (type uint8_t) refer to eTeamColor for value assignment
    def CvCmd_GetTeamColor(self):
        return self.teamColor

    # @param[out]: (type uint8_t) auto aim mode on or off
    def CvCmd_GetAutoAimMode(self):
        return self.AutoAimSwitch

    # @param[out]: (type uint8_t) auto move mode on or off
    def CvCmd_GetAutoMoveMode(self):
        return self.AutoMoveSwitch

    # @param[out]: (type uint8_t) enemy mode on or off
    def CvCmd_GetEnemyMode(self):
        return self.EnemySwitch
    
    def CvCmd_RedOutpostHP(self):
        return self.RedOutpostHP
    
    def CvCmd_BlueOutposeHP(self):
        return self.BlueOutpostHP
    
    def CvCmd_StartShoot(self):
        # ShootSwitch will be automatically disabled after SHOOT_TIMEOUT_SEC
        self.ShootSwitch = True
        self.shootStartTime = time.time()
    
    def CvCmd_SetChassisSpinningSwitch(self, spinningSwitch):
        self.ChassisSpinningSwitch = spinningSwitch

    # @param[out]: (type fp32, unit rad) gimbal absolute pitch angle
    def CvCmd_GetGimbalPitch(self):
        return self.gimbal_pitch_angle
    
    # @param[out]: (type fp32, unit rad) gimbal absolute yaw angle
    def CvCmd_GetGimbalYaw(self):
        # Yaw angles from cv and control board are negative to each other, as used in CvCmd_ConditionSignals
        return -self.gimbal_yaw_angle
    
    # @brief main API function
    # @param[in] chassis_speed_x and chassis_speed_y: type is float; can be positive/negative; will be converted to float (32 bits)
    def CvCmd_Heartbeat(self, gimbal_pitch_target, gimbal_yaw_target, chassis_speed_x, chassis_speed_y):
        self.CvCmd_ConditionSignals(gimbal_pitch_target, gimbal_yaw_target, chassis_speed_x, chassis_speed_y)
        self.CvCmd_TxHeartbeat()
        # Rx
        fRxFinished = False
        while fRxFinished == False:
            fRxFinished = self.CvCmd_RxHeartbeat()
    
    def CvCmd_InfoReqManager(self):
        if ((time.time() - self.gimbalStatusUpdateTime) >= 0.005):
            self.infoRequestPending |= self.eInfoBits.MODE_GIMBAL_ANGLE_BIT.value
            if self.DEBUG_CV:
                print("Angle Update Time Delay", time.time() - self.gimbalStatusUpdateTime)
            self.gimbalStatusUpdateTime = time.time()
            
        if ((time.time() - self.refStatusUpdateTime) >= 0.1):
            self.infoRequestPending |= self.eInfoBits.MODE_REF_STATUS_BIT.value
            if self.DEBUG_CV:
                print("Ref Update Time Delay",time.time() - self.refStatusUpdateTime)
            self.refStatusUpdateTime = time.time()

    def CvCmd_ConditionSignals(self, gimbal_pitch_target, gimbal_yaw_target, chassis_speed_x, chassis_speed_y):
        if self.DEBUG_CV:
            print("gimbal_yaw_target: ", gimbal_yaw_target, "gimbal_pitch_target: ", gimbal_pitch_target)

        # Gimbal: pixel to angle conversion
        # TODO: Use parabolic instead of linear trajectory
        # CV positive directions: +x is to the right, +y is downwards
        # angle unit is radian
        self.gimbal_cmd_pitch = np.float32(gimbal_pitch_target)
        self.gimbal_cmd_yaw = np.float32(-gimbal_yaw_target)

        # Chassis: speed to speed conversion
        # CV positive directions: +x is to the right, +y is upwards
        # Remote controller positive directions: +x is upwards, +y is to the left
        self.chassis_cmd_speed_x = np.float32(chassis_speed_y)
        self.chassis_cmd_speed_y = -np.float32(chassis_speed_x)

    def CvCmd_RxHeartbeat(self):
        fRxFinished = True

        if self.DEBUG_CV:
            if self.ser.is_open and self.ser.in_waiting > 0:
                print("Input buffer size: ", self.ser.in_waiting)
            if self.prev_Rx_State != self.Rx_State:
                print("Rx_State: ", self.Rx_State)
                self.prev_Rx_State = self.Rx_State

        if self.Rx_State == self.eRxState.RX_STATE_INIT:
            if not self.ser.is_open:
                self.ser.open()
            self.CvCmd_Reset()
            self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_PKG
            fRxFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_WAIT_FOR_PKG:
            if self.ser.in_waiting >= self.DATA_PACKAGE_SIZE:
                bytesRead = self.ser.read(self.ser.in_waiting)
                # @TODO: use regex to search msg by msg instead of processing only the last msg. For now, control board don't have much to send, so it's fine.
                setModeRequestPackets = re.findall(self.eSepChar.CHAR_HEADER.value + b".." + self.eMsgType.MSG_MODE_CONTROL.value + b"." + self.eSepChar.CHAR_UNUSED.value + b"{15}", bytesRead)
                infoFeedbackPackets = re.findall(self.eSepChar.CHAR_HEADER.value + b".." + self.eMsgType.MSG_INFO_FEEDBACK.value + b"." + b".." + b".........." + self.eSepChar.CHAR_UNUSED.value + b"{3}", bytesRead)
                if self.DEBUG_CV:
                    print("bytesRead: ", bytesRead)
                    print("setModeRequestPackets: ", setModeRequestPackets)
                    print("infoFeedbackPackets: ", infoFeedbackPackets)

                if setModeRequestPackets:
                    # read the mode of the last packet, because it's the latest
                    rxSwitchBuffer = setModeRequestPackets[-1][self.DATA_PAYLOAD_INDEX]

                    self.AutoAimSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_AIM_BIT.value)
                    self.AutoMoveSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_AUTO_MOVE_BIT.value)
                    self.EnemySwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_ENEMY_DETECTED_BIT.value)
                    # The following switches are controlled by CV
                    # self.ShootSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_SHOOT_BIT.value)
                    # self.ChassisSpinningSwitch = bool(rxSwitchBuffer & self.eModeControlBits.MODE_CHASSIS_SPINNING_BIT.value)

                    self.ackMsgInfo["reqCtrlTimestamp"] = struct.unpack('<H', setModeRequestPackets[-1][self.DATA_TIMESTAMP_INDEX:self.DATA_TIMESTAMP_INDEX+2])[0]
                    self.ackMsgInfo["reqRxTimestamp"] = self.CvCmd_GetUint16Time()
                    # self.cvCmdCount = 0
                    self.Rx_State = self.eRxState.RX_STATE_SEND_ACK
                    fRxFinished = False

                if infoFeedbackPackets:
                    for packet in infoFeedbackPackets:
                        rxInfoType = packet[self.DATA_PAYLOAD_INDEX]
                        rxInfoData = packet[self.DATA_PAYLOAD_INDEX+1:]
                        if rxInfoType == self.eInfoBits.MODE_TRAN_DELTA_BIT.value:
                            # Warning: tranDelta is signed
                            self.tranDelta = struct.unpack_from('<h', rxInfoData, 0)[0]
                            self.infoRequestPending &= ~rxInfoType
                            if self.DEBUG_CV:
                                print("tranDelta: ", self.tranDelta)
                        elif rxInfoType == self.eInfoBits.MODE_CV_SYNC_TIME_BIT.value:
                            self.CvSyncTime = struct.unpack_from('<H', rxInfoData, 0)[0]
                            self.infoRequestPending &= ~rxInfoType
                            if self.DEBUG_CV:
                                print("CvSyncTime: ", self.CvSyncTime)
                        elif rxInfoType == self.eInfoBits.MODE_REF_STATUS_BIT.value:
                            (self.game_progress, self.teamColor, self.time_remain, self.current_HP, self.RedOutpostHP, self.BlueOutpostHP) = struct.unpack_from('<BBHHHH', rxInfoData, 0)
                            self.infoRequestPending &= ~rxInfoType
                            if self.DEBUG_CV:
                                print("RefStatus: ", self.game_progress, self.teamColor, self.time_remain, self.current_HP, self.RedOutpostHP, self.BlueOutpostHP)
                        elif rxInfoType == self.eInfoBits.MODE_GIMBAL_ANGLE_BIT.value:
                            (self.gimbal_yaw_angle, self.gimbal_pitch_angle) = struct.unpack_from('<ff', rxInfoData, 0)
                            self.infoRequestPending &= ~rxInfoType
                            if self.DEBUG_CV:
                                print("GimbalAngle: ", self.gimbal_yaw_angle, self.gimbal_pitch_angle)
                    # Do not change Rx_State or fRxFinished

                # No valid msg received, retry connection
                if not (setModeRequestPackets or infoFeedbackPackets):
                    self.ser.reset_input_buffer()
                    fRxFinished = True

        elif self.Rx_State == self.eRxState.RX_STATE_SEND_ACK:
            if (time.time() - self.prevTxTime >= self.MIN_TX_SEPARATION_SEC):
                # Timestamps
                self.CvSyncTime = self.CvCmd_GetUint16Time()
                uiExecDelta = self.CvCmd_GetUint16Delta(self.CvSyncTime, self.ackMsgInfo["reqRxTimestamp"])
                self.txAckMsg[self.txAckMsgPayloadIndex:self.txAckMsgPayloadIndex+6] = struct.pack('<HHH', self.ackMsgInfo["reqCtrlTimestamp"], uiExecDelta, self.CvSyncTime)

                self.CvCmd_BuildSendTxMsg(self.txAckMsg)

                # Trigger info request in TxHeartbeat
                self.infoRequestPending |= self.eInfoBits.MODE_TRAN_DELTA_BIT.value | self.eInfoBits.MODE_CV_SYNC_TIME_BIT.value
                self.Rx_State = self.eRxState.RX_STATE_WAIT_FOR_PKG
            fRxFinished = True

        return fRxFinished

    def CvCmd_TxHeartbeat(self):
        # Tx: keeping sending cmd to keep control board alive (watchdog timer logic)
        if (time.time() - self.prevTxTime >= self.MIN_TX_SEPARATION_SEC):
            if self.infoRequestPending and (time.time() - self.prevInfoReqTime > self.MIN_INFO_REQ_SEPARATION_SEC):
                self.txInfoRequestMsg[self.DATA_PAYLOAD_INDEX] = self.infoRequestPending
                self.CvCmd_BuildSendTxMsg(self.txInfoRequestMsg)
                self.prevInfoReqTime = time.time()
            elif self.ChassisSpinningSwitch != self.prevChassisSpinningSwitch:
                self.prevChassisSpinningSwitch = self.ChassisSpinningSwitch
                self.CvCmd_BuildCvControlledTxSetModeMsg()
                self.CvCmd_BuildSendTxMsg(self.txSetModeMsg)
                if self.DEBUG_CV:
                    print("Auto spinning " + ("on" if self.prevChassisSpinningSwitch else "off"))
            elif ((self.AutoAimSwitch or self.AutoMoveSwitch) and (self.tranDelta != None)):
                self.txCvCmdMsg[self.DATA_PAYLOAD_INDEX:self.DATA_PAYLOAD_INDEX+16] = struct.pack('<ffff', self.gimbal_cmd_yaw, self.gimbal_cmd_pitch, self.chassis_cmd_speed_x, self.chassis_cmd_speed_y)
                self.CvCmd_BuildSendTxMsg(self.txCvCmdMsg)
                self.CvCmd_InfoReqManager()
                # self.cvCmdCount += 1
                # if (self.cvCmdCount % 10 == 0):
                #     self.infoRequestPending |= self.eInfoBits.MODE_TRAN_DELTA_BIT.value

        # Latching shoot switch logic
        if self.ShootSwitch:
            if time.time() - self.prevTxTime >= self.MIN_TX_SEPARATION_SEC:
                self.CvCmd_BuildCvControlledTxSetModeMsg()
                self.CvCmd_BuildSendTxMsg(self.txSetModeMsg)

            if time.time() - self.shootStartTime > self.SHOOT_TIMEOUT_SEC:
                # control should automatically disable itself as well
                self.ShootSwitch = False