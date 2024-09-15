import matplotlib.pyplot as plt
import serial

class imu:
    def __init__(self,port):
        
        self.ACCData=[0.0]*58        
        self.FrameState = 0        
        self.Bytenum = 0             

        #Header 
        SOF = 0xa5
        data_length = 30 
        self.seq = 0x00 
        crc8 = 0x00 
        cmd_id = 0x0302 
        key_1 = 0xff
        key_2 = 0xff
        blank = 0x0001

        a = [0.0]*3
        w = [0.0]*3
        m = [0.0]*3

        Angle = [0.0]*3

        kz=[0.0]*3    #输入的控制变量1,2

        q=[0.0]*4  
        end_float = float("inf")
        self.ser = serial.Serial(port, 115200)

    def DataRead(self):   

        inputdata = self.ser.read(33)

        self.seq = self.seq + 1  
        if (self.seq > 255):
            self.seq = 0
    
        for data in inputdata:  

            if self.FrameState==0:  

                if data==0x50 : 

                    self.Bytenum=0
                    self.FrameState=1
                    continue    

            elif self.FrameState==1: # acc   

                if self.Bytenum<57:            
                    self.ACCData[self.Bytenum+1]=data 
                    self.Bytenum+=1

                else:
                    if (self.ACCData[49] * 256.0 * 256.0 + self.ACCData[50] * 256.0 + self.ACCData[51] - 1000000) * 0.001 == 128 :
                        self.Bytenum=0

                        self.FrameState=0
            
 

    def get_gyro(self):   # angular acceleration

        gyro_x = (self.ACCData[1] * 256.0 * 256.0 + self.ACCData[2] * 256.0 + self.ACCData[3] - 1000000) * 0.001; # gx
        gyro_y = (self.ACCData[4] * 256.0 * 256.0 + self.ACCData[5] * 256.0 + self.ACCData[6] - 1000000) * 0.001; # gy
        gyro_z = (self.ACCData[7] * 256.0 * 256.0 + self.ACCData[8] * 256.0 + self.ACCData[9] - 1000000) * 0.001; # gz

        return gyro_x, gyro_y, gyro_z

    def get_acc(self): #axis accelerator

        acc_x = (self.ACCData[10] * 256.0 * 256.0 + self.ACCData[11] * 256.0 + self.ACCData[12] - 1000000) * 0.001;  # ax
        acc_y = (self.ACCData[13] * 256.0 * 256.0 + self.ACCData[14] * 256.0 + self.ACCData[15] - 1000000) * 0.001;  # ay
        acc_z = (self.ACCData[16] * 256.0 * 256.0 + self.ACCData[17] * 256.0 + self.ACCData[18] - 1000000) * 0.001;  # az

        return acc_x, acc_y, acc_z

    def get_mg(self): #magnetometer

        mg_x = (self.ACCData[19] * 256.0 * 256.0 + self.ACCData[20] * 256.0 + self.ACCData[21] - 1000000) * 0.001;  # mx
        mg_y = (self.ACCData[22] * 256.0 * 256.0 + self.ACCData[23] * 256.0 + self.ACCData[24] - 1000000) * 0.001;  # my
        mg_z = (self.ACCData[25] * 256.0 * 256.0 + self.ACCData[26] * 256.0 + self.ACCData[27] - 1000000) * 0.001;  # mz

        return mg_x, mg_y, mg_z

    def get_angle(self): #euler angle

        angle_x = (self.ACCData[28] * 256.0 * 256.0 + self.ACCData[29] * 256.0 + self.ACCData[30] - 1000000) * 0.001;  # roll angle_x
        angle_y = (self.ACCData[31] * 256.0 * 256.0 + self.ACCData[32] * 256.0 + self.ACCData[33] - 1000000) * 0.001;  # pitch
        angle_z = (self.ACCData[34] * 256.0 * 256.0 + self.ACCData[35] * 256.0 + self.ACCData[36] - 1000000) * 0.001;  # yaw

        return angle_x,angle_y,angle_z

 

    def check_angle(cur, pre):

        change = cur - pre
        
        delta_r, delta_p, delta_yaw = change
        print(delta_r,delta_p, delta_yaw, end = "\r")
    

    def get_kz(self):
        kz_x = (self.ACCData[52] * 256.0 * 256.0 + self.ACCData[53] * 256.0 + self.ACCData[54] - 1000000) * 0.001;  # 1
        kz_y = (self.ACCData[55] * 256.0 * 256.0 + self.ACCData[56] * 256.0 + self.ACCData[57] - 1000000) * 0.001;  #2
        kz_z = 0;  #

        return kz_x,kz_y,kz_z

 
    def get_q(self):

        q_0 = (self.ACCData[37] * 256.0 * 256.0 + self.ACCData[38] * 256.0 + self.ACCData[39] - 1000000) * 0.001;  # q0
        q_1 = (self.ACCData[40] * 256.0 * 256.0 + self.ACCData[41] * 256.0 + self.ACCData[42] - 1000000) * 0.001;  #q1
        q_2 = (self.ACCData[43] * 256.0 * 256.0 + self.ACCData[44] * 256.0 + self.ACCData[45] - 1000000) * 0.001;  #q2
        q_3 = (self.ACCData[46] * 256.0 * 256.0 + self.ACCData[47] * 256.0 + self.ACCData[48] - 1000000) * 0.001;  #q3

        return q_0,q_1,q_2,q_3


