import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math
import random

class SensorPublisher(Node):
    """
    A ROS 2 node that publishes x,y and velocity.

    This node demonstrates the publisher pattern in ROS2, where messages are
    continuously sent to a topic that other nodes (like turtlesim) can subscribe to. 
    """
    
    def __init__(self):
        # Initialize the node
        super().__init__('sensor_publisher')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vel = 2.0
        self.prev_v = 2.0
        self.angle = 1.0
        self.clock = self.get_clock().now()
        
        # Create a publisher
        # - Topic: '/nav_msgs/Odometry' (turtlesim's x,y and velocity command topic)
        # - Message type: Odometry (contains x, y, linear_velocity, angular_velocity)
        # - Queue size: 10
        self.publisher_nav = self.create_publisher(
            Odometry,
            '/nav_msgs/Odometry',
            10
        )

        # Create a publisher
        # - Topic: '/sensor_msgs/imu'
        # - Message type: Imu
        # - Queue size: 10
        self.publisher_sensor = self.create_publisher(
            Imu,
            '/sensor_msgs/imu',
            10
        )

        # Create a timer that calls timer_callback every 0.5 seconds
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.get_logger().info('Sensor publisher has started!')
    
    def timer_callback(self):
        """
        This function is called every 0.5 seconds by the timer.
        It creates and publishes an odome message and an imu message.
        """
        
        now = self.get_clock().now()
        dt = (now - self.clock).nanoseconds * 1e-9
        self.clock = now

        noise_vel = random.gauss(0.0,self.vel/10)
        noise_angle = random.gauss(0.0,self.angle/10)
        self.x += (self.vel + noise_vel) * math.cos(self.theta) * dt
        self.y += (self.vel + noise_vel) * math.sin(self.theta) * dt
        self.theta += (self.angle + noise_angle) * dt

        # Create a new Odometry message and set the values
        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.twist.twist.linear.x = self.vel + noise_vel
        odom.twist.twist.angular.z = self.angle + noise_angle

        # Publish the message
        self.publisher_nav.publish(odom)

        # Log the message
        self.get_logger().info(
            f'Nav Publishing - X: {self.x:.2f}, Y: {self.y:.2f}, Linear velocity: {odom.twist.twist.linear.x:.2f}, Angular velocity: {odom.twist.twist.angular.z:.2f}'
        )

        # Create a new Imu message and set the values
        imu = Imu()
        quater = quaternion_from_euler(0.0,0.0,self.theta)
        imu.orientation = Quaternion(
            x = quater[0],
            y = quater[1],
            z = quater[2],
            w = quater[3]
        )
        imu.angular_velocity.z = self.angle + noise_angle
        imu.linear_acceleration.x = (self.vel + noise_vel - self.prev_v)/dt
        self.prev_v = self.vel + noise_vel

        # Publish the message  and log the message
        self.publisher_sensor.publish(imu)
        self.get_logger().info(
            f'Sensor Publishing - Orientation:{imu.orientation}, Angular velocity:{imu.angular_velocity.z}, Linear acceleration:{imu.linear_acceleration.x}'
        )
def main(args=None):
    #Initialize the ROS 2 Python client library 
    rclpy.init(args=args)

    # Create an instance of our node
    sensor_publisher = SensorPublisher()
    
    # Keep the node running and processing callbacks
    # This will run until you press Ctrl + C
    rclpy.spin(sensor_publisher)
    
    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
