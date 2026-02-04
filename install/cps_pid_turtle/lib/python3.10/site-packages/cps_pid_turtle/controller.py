import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import math 
import random
from turtlesim.msg import Pose

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.pose = None
        self.goalx = [7.0,6.0,5.0]
        self.goaly = [5.544,5.544 + math.sqrt(3),5.544]
        self.theta = 0
        self.side = 0
        self.threshold = 0.05
        self.kp_dist = 0.5
        self.kp_ang = 2
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.subscriber_ = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.get_logger().info('Velocity Publisher has started!')
    
    def pose_callback(self,msg):
        self.pose = msg

    def timer_callback(self):
        if self.pose is None:
            return
        sensor_x = self.pose.x
        sensor_y = self.pose.y

        diff_x = self.goalx[self.side] - sensor_x
        diff_y = self.goaly[self.side] - sensor_y
        diff = math.sqrt(diff_x ** 2 + diff_y ** 2) 
        angle_diff = math.atan2(diff_y,diff_x) - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        action = Twist()
        action.linear.x = self.kp_dist * diff
        action.angular.z = self.kp_ang * angle_diff

        if diff < self.threshold:
            self.side += 1
            action.linear.x = 0.0
            if self.side >= 3:
                self.side = 0

        self.publisher_.publish(action)
        self.get_logger().info(f'Publishing: linear.x={self.kp_dist} angular.z={action.angular.z}')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our node
    velocity_publisher = VelocityPublisher()
    
    # Keep the node running and processing callbacks
    # This will run until you press Ctrl+C
    rclpy.spin(velocity_publisher)
    
    # Cleanup
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
