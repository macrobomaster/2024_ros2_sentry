import camera_cv.CvCmdApi as CvCmdApi
import time
import camera_cv.CvHelper as CvHelper
import cv2
import numpy as np

class Target:

    def __init__(self):
        self.target_angle = [0, 0, 0]
        self.enemy_detected = False
        self.Global_xyz_filtered = [0, 0, 0]
        self.CvCmder = CvCmdApi.CvCmdHandler('/dev/ttyTHS1')
        self.pitch_lower_limit = 0
        self.cam_started = False
        self.last_seen = -2
        self.color = self.CvCmder.CvCmd_GetTeamColor()
   
    def set_target_angle(self, target_angle):
        self.last_seen = time.time()
        self.target_angle = target_angle
    
    def set_pitch_lower_limit(self, lower_limit):
        self.pitch_lower_limit = lower_limit
        
    def is_detected(self, enemy_detected):
        self.enemy_detected = enemy_detected

    def ack_start(self,flag):
        self.cam_started = True

    def get_target_angle(self):
        return self.target_angle

    def get_enemy_detected(self):
        return self.enemy_detected

    def is_enemy(self,crop):
        crop=cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        # Define color ranges for red and blue
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        lower_blue = np.array([100, 120, 70])
        upper_blue = np.array([130, 255, 255])

        # Create masks for each color range
        red_mask = cv2.inRange(crop, lower_red, upper_red)
        blue_mask = cv2.inRange(crop, lower_blue, upper_blue)

        # Count the number of pixels for each color
        red_pixels = cv2.countNonZero(red_mask)
        blue_pixels = cv2.countNonZero(blue_mask)

        # Determine the dominant color
        if red_pixels > blue_pixels:
            Target = 1   # Red
        elif blue_pixels > red_pixels:
            Target = 0   # Blue

        if Target != self.color:
            return True
    

    # Heartbeat from CV to Control
    def call_heartBeat(self):
        while True:
            self.color = self.CvCmder.CvCmd_GetTeamColor()

            # Limit the pitch angle to the lower limit
            if self.target_angle[1] < self.pitch_lower_limit:
                self.target_angle[1] =  self.pitch_lower_limit

            if self.enemy_detected:
                self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target=self.target_angle[1], gimbal_yaw_target=self.target_angle[2], chassis_speed_x=0, chassis_speed_y=0)
                # self.CvCmder.CvCmd_Shoot()
            else:
                if self.cam_started and ((time.time()-self.last_seen) >=1.5):
                    # print("search target")
                    self.target_angle[2] += 0.003
                    self.target_angle[1] = -0.2

                self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target=self.target_angle[1], gimbal_yaw_target=self.target_angle[2], chassis_speed_x=0, chassis_speed_y=0)
            
            time.sleep(1/400)