import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32,String
import math
import CvCmd_bridge.CvCmdApi as CvCmdApi
from std_msgs.msg import Float32MultiArray

class CmdVelToImu(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_imu')

        self.subscription_chasis = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10)
        self.subscription_chasis

        self.subscription_gimbal = self.create_subscription(
            Twist, 
            'cmd_angle', 
            self.imu_callback, 
            10)
        self.subscription_gimbal
        
        # Store the latest messages
        self.latest_cmd_vel = None
        self.latest_imu = None

        self.CvCmder = CvCmdApi.CvCmdHandler('/dev/ttyTHS1')

        self.red_outposthp = 400.0
        self.blue_outposthp = 400.0
        self.current_hp = 400.0

        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu_angles', 100)



    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.process_and_publish()

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.process_and_publish()
    
    def process_and_publish(self):
        # Ensure we have received both messages before processingo r self.latest_imu is None
        # if self.latest_cmd_vel is None :
        #     return
        print(self.CvCmder.CvCmd_GetGameProgress())
        imu_angles_msg = Float32MultiArray()

        self.red_outposthp = float(self.CvCmder.CvCmd_RedOutpostHP())
        self.blue_outposthp = float(self.CvCmder.CvCmd_BlueOutposeHP())
        self.current_hp = float(self.CvCmder.CvCmd_GetCurrentHp())

        if self.CvCmder.CvCmd_GetTeamColor() == 0:
            #Current on blue Team
            #Enemy Outpost, Our Outpost, Current HP
            imu_angles_msg.data = [self.red_outposthp, self.blue_outposthp ,self.current_hp]
        else:
            imu_angles_msg.data = [ self.blue_outposthp, self.red_outposthp, self.current_hp]
        
        self.publisher_.publish(imu_angles_msg)

        if (self.CvCmder.CvCmd_GetGameProgress() == 1):   
            # Extract data from cmd_vel
            linear_x = -self.latest_cmd_vel.linear.y
            linear_y = self.latest_cmd_vel.linear.x
            
        else:
            linear_x,linear_y = 0,0

        self.CvCmder.CvCmd_SetChassisSpinningSwitch(True)
        
        # Extract data from imu
        # imu_orientation = self.latest_imu.orientation
        # imu_angle = math.atan2(imu_orientation.y, imu_orientation.x)  # example calculation

        # Create a combined message
        combined_info = f"CmdVel: linear_x={linear_x}, linear_y={linear_y}"
        
        self.CvCmder.CvCmd_Heartbeat(gimbal_pitch_target= 0 , gimbal_yaw_target= 0, chassis_speed_x=linear_x, chassis_speed_y=linear_y)

        # Publish the combined message
        # msg = String()
        # msg.data = combined_info
        # self.publisher_.publish(msg)

        # Log the combined information
        self.get_logger().info(combined_info)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_to_imu = CmdVelToImu()
    rclpy.spin(cmd_vel_to_imu)
    cmd_vel_to_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

