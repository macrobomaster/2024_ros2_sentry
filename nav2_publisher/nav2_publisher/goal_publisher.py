import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'imu_angles',
            self.imu_angles_callback,
            1
        )

        self.goals = [
            {"position": {"x": -3.4, "y": -4.2, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}},
            ]

        self.home = [
            {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}}
        ]

        self.Enemy_Outpost_Location = [
            {"position": {"x": 5.0, "y": 6.7, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}}
        ]

        self.current_goal_index = 0

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.Enemy_OutpostHp = 400
        self.Our_OutpostHp = 400
        self.Current_Hp = 400

    def imu_angles_callback(self, msg):
        
        info = msg.data
        self.Enemy_OutpostHp = info[0]
        self.Our_OutpostHp = info[1]
        self.get_logger().info(f'hp: {self.Current_Hp}')
        self.Current_Hp = info[2]
        

    def timer_callback(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        if self.Our_OutpostHp <= 0 or self.Current_Hp <= 150:
            goal = self.home[0]
            goal_pose.pose.position.x = goal["position"]["x"]
            goal_pose.pose.position.y = goal["position"]["y"]
            goal_pose.pose.position.z = goal["position"]["z"]
            goal_pose.pose.orientation.x = goal["orientation"]["x"]
            goal_pose.pose.orientation.y = goal["orientation"]["y"]
            goal_pose.pose.orientation.z = goal["orientation"]["z"]
            goal_pose.pose.orientation.w = goal["orientation"]["w"]

        elif self.Enemy_OutpostHp > 0:
            goal = self.Enemy_Outpost_Location[0]
            goal_pose.pose.position.x = goal["position"]["x"]
            goal_pose.pose.position.y = goal["position"]["y"]
            goal_pose.pose.position.z = goal["position"]["z"]
            goal_pose.pose.orientation.x = goal["orientation"]["x"]
            goal_pose.pose.orientation.y = goal["orientation"]["y"]
            goal_pose.pose.orientation.z = goal["orientation"]["z"]
            goal_pose.pose.orientation.w = goal["orientation"]["w"]

        else:

            goal = self.goals[self.current_goal_index]
        
            goal_pose.pose.position.x = goal["position"]["x"]
            goal_pose.pose.position.y = goal["position"]["y"]
            goal_pose.pose.position.z = goal["position"]["z"]
            goal_pose.pose.orientation.x = goal["orientation"]["x"]
            goal_pose.pose.orientation.y = goal["orientation"]["y"]
            goal_pose.pose.orientation.z = goal["orientation"]["z"]
            goal_pose.pose.orientation.w = goal["orientation"]["w"]

            self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
            goal = self.Enemy_Outpost_Location[0]

        self.publisher_.publish(goal_pose)
        self.get_logger().info(f'Publishing goal_pose: {goal_pose}')
        

def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    try:
        rclpy.spin(goal_publisher)
    except KeyboardInterrupt:
        pass

    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
