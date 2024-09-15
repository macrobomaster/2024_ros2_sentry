import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import math

import camera_cv.CvHelper as CvHelper
from std_msgs.msg import Float32,String
import camera_cv.target as target
import camera_cv.CvCmdApi as CvCmdApi
from geometry_msgs.msg import Twist
import time

import camera_cv.imu_data as imu_data

import matplotlib.pyplot as plt
from filterpy.kalman import ExtendedKalmanFilter as EKF 
from filterpy.common import Q_discrete_white_noise
import ros2_numpy

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            1
        )

        self.subscription_chasis = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10)
        self.subscription_chasis

        # prevent unused variable warning 
        self.subscription 
        # self.imu_subscriber 

        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu_angles', 100)

        # Initialize variables to store the latest messages
        self.latest_image = None
        self.latest_imu = None
        self.model = YOLO("/home/macrm/cam_ws/src/camera_cv/camera_cv/best.engine")

        self.CvCmder = CvCmdApi.CvCmdHandler('/dev/ttyTHS1')
        self.cur_time = time.time()

        last = [0,0,0]

        self.Global_xyz = np.array([0.0,0.0,0.0])

        angle_record = []
        location_record = []
        theta_record = []
        target_record = []
        self.cur_angle = [0,0,0]
        Global_xyz_filtered = [0,0,0]

        target_yaw_record = np.zeros(1)
        target_pitch_record = np.zeros(1)
        yaw_record = np.zeros(1)

        self.cur_time = time.time()

        # Initialize the EKF
        self.ekf = EKF(dim_x=2, dim_z=1)

        # Initial state
        self.ekf.x = np.array([0.0, 0.0])  # Starting with angle 0 and small angular velocity
        self.ekf.F = np.eye(2)             # State transition matrix (will be updated)
        self.ekf.R = np.array([[0.2]])     # Measurement noise covariance
        self.ekf.Q = Q_discrete_white_noise(dim=2, dt=15, var=0.5)  # Process noise covariance
        self.ekf.P = 10       # Initial state covariance

        self.frame_count = 0
        self.boundry = [[0,0],[0,0]]
        last_center = [0,0]
        self.detection_dist = np.array([5])
        self.linear_x,self.linear_y = 0,0

    def reset_ekf(self):
        # Initial state
        self.ekf.x = np.array([self.cur_angle[2], 0.0])  # Starting with angle 0 and small angular velocity
        self.ekf.F = np.eye(2)             # State transition matrix (will be updated)
        self.ekf.R = np.array([[0.2]])     # Measurement noise covariance
        self.ekf.Q = Q_discrete_white_noise(dim=2, dt=15, var=0.5)  # Process noise covariance
        self.ekf.P = 10       # Initial state covariance

    def cmd_vel_callback(self, msg):

        self.latest_cmd_vel = msg        

        if (self.CvCmder.CvCmd_GetGameProgress() == 1):   
            # Extract data from cmd_vel
            self.linear_x = -self.latest_cmd_vel.linear.y
            self.linear_y = self.latest_cmd_vel.linear.x
        else:
            self.linear_x,self.linear_y = 0,0
        
        self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target= self.Global_xyz[1],
                                    gimbal_yaw_target= self.Global_xyz[2],
                                    chassis_speed_x=self.linear_x, 
                                    chassis_speed_y=self.linear_y)

    def listener_callback(self, msg):
        
        self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target= self.Global_xyz[1] , gimbal_yaw_target= self.Global_xyz[2] ,  chassis_speed_x=self.linear_x, chassis_speed_y=self.linear_y)
        self.CvCmder.CvCmd_SetChassisSpinningSwitch(True)

        imu_angles_msg = Float32MultiArray()
        
        print(np.float32(self.CvCmder.CvCmd_RedOutpostHP()),
                                   np.float32(self.CvCmder.CvCmd_BlueOutposeHP()),
                                   np.float32(self.CvCmder.CvCmd_GetCurrentHp()))
        
        red_outposthp = float(self.CvCmder.CvCmd_RedOutpostHP())
        blue_outposthp = float(self.CvCmder.CvCmd_BlueOutposeHP())
        current_hp = float(self.CvCmder.CvCmd_GetCurrentHp())

        if self.CvCmder.CvCmd_GetTeamColor() == 0:
            #Current on blue Team
            #Enemy Outpost, Our Outpost, Current HP
            imu_angles_msg.data = [red_outposthp, blue_outposthp, current_hp]
        else:
            imu_angles_msg.data = [ blue_outposthp, red_outposthp, current_hp]
        
        self.publisher_.publish(imu_angles_msg)

        dt = time.time() - self.cur_time

        self.detection_dist = []
        
        #frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Convert ROS Image message to OpenCV format
        frame = ros2_numpy.numpify(msg)
        frame = cv2.resize(cv2.rotate(frame, cv2.ROTATE_180),(640,480))
        frame  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        self.cur_time = time.time()

        results = self.model.track(frame,verbose=False,device="cuda",batch=16,persist=True,task="detect",conf=0.9)[0]

        process_time = time.time()-self.cur_time

        self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target= self.Global_xyz[1] , gimbal_yaw_target= self.Global_xyz[2] , chassis_speed_x=self.linear_x, chassis_speed_y=self.linear_y)
        

        self.cur_angle[2] = CvHelper.wrap_angle(np.float32(self.CvCmder.CvCmd_GetGimbalYaw()))
        self.cur_angle[1] = np.float32(self.CvCmder.CvCmd_GetGimbalPitch())
        
        print(self.cur_angle)

        for result in results:
            if result != None:
                

                detected = True
                boxes_xy = result.boxes.xyxy[0]
                
                center_x = int((boxes_xy[0]+boxes_xy[2])/2)
                center_y = int((boxes_xy[1]+boxes_xy[3])/2)        

                delta_x =  abs(boxes_xy[0]-boxes_xy[2])
                delta_y = abs(boxes_xy[1]-boxes_xy[3])

                dist = math.sqrt(center_x**2 + center_y**2)

                combined_info = f"Target location center_x={center_x},
                                 center_y={center_y},process_time={dt},
                                 angular_velocity={self.ekf.x[1]},
                                 shoot = {self.CvCmder.ShootSwitch},
                                 process_time={process_time}"
                
                focal_length_in_pixel = 1440*1.5* 4.81 / 11.043412 #<-- tune this euqation to fit camera config 
                focal_length_in_pixel_pitch =  3.1*1080 * 4.81 / 11.043412 #<-- tune this euqation to fit camera config
                
                #calculate Delta Angle
                theta  = math.atan((center_x - 320)/focal_length_in_pixel)
                phi = -math.atan((center_y - 240)/focal_length_in_pixel_pitch)

                self.detection_dist.append(math.sqrt(theta**2 + phi**2))

                if ((self.boundry[0][0]-1*delta_x) < center_x < (self.boundry[1][0]+1*delta_x)) and ((self.boundry[0][1] -1*delta_y)< center_y < (self.boundry[1][1]+1*delta_x)):
                    #anti-bayblading, if swithcing to new target it wn't overshoot
                    self.ekf.F = CvHelper.jfx(self.ekf.x, dt/30)
                    self.ekf.update(np.array(self.cur_angle[2]+1.5*theta), HJacobian=CvHelper.jhx, Hx=CvHelper.hx)
                    self.ekf.predict()
                    self.ekf.x[0] = CvHelper.normalize_angle(self.ekf.x[0])  # Normalize the angle
                    final_target = self.ekf.x[0]

                else:
                    self.reset_ekf()
                    final_target = self.cur_angle[2]+theta

                self.Global_xyz[2] = final_target
                #this pitch angle is with distance compensations
                self.Global_xyz[1] = self.cur_angle[1]-phi-(480/(delta_y*450))
                
                #update boundary
                self.boundry = [[boxes_xy[0],boxes_xy[1]],
                                [boxes_xy[2],boxes_xy[3]]]

                # Log the combined information
                self.get_logger().info(combined_info)

                frame = cv2.circle(frame, (center_x,center_y), 10, (255, 0, 0) , 2)

        self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target= self.Global_xyz[1],
                                    gimbal_yaw_target= self.Global_xyz[2],  
                                    chassis_speed_x=self.linear_x, 
                                    chassis_speed_y=self.linear_y)
        self.CvCmder.CvCmd_StartShoot()
        
        # Display the image
        cv2.imshow("Image Window", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

