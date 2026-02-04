import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import math 
import random
from turtlesim.msg import Pose

class TurtleController(Node):
    """
    A ROS 2 node that publishes distance to the destination and the angle
    to make the turtle move in a triangle. 

    This node demonstrates the publisher  pattern in ROS 2,
    where messages are continuously sent to a topic that other nodes
    can subscribe to.

    And also subscribe to the turtle's pose and prints it.
    This node demonstrates the subscriber pattern in ROS 2 where a node
    listens to message published on a topic and processes them in a callback.
    """

    def __init__(self):
        #Initialize the node with a name
        super().__init__('turtle_controller')
        self.pose = None
        self.goalx = [7.0,6.0,5.0]
        self.goaly = [5.544,5.544 + math.sqrt(3),5.544]
        self.theta = 0
        self.side = 0
        self.threshold = 0.05
        self.kp_dist = 0.5
        self.kp_ang = 2

        # Create a publisher
        # - Topic: '/turtle1/cmd_vel' 
        # - Message type: Twist (contains linear and angular velocity)
        # - Queue size: 10 (number of message to buffer)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        
        # Create a subscriber
        # - Topic: 'turtle1/pose' (turtlesim publishes pose here)
        # - Message type: Pose (contains x, y, theta, linear_velocity, 
        #                       angular_velocity)
        # - Callback function: pose_callback (called when a message 
        #                                      is received)
        # -Queue size: 10
        self.subscriber_ = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)

        # Create a timer that calls timer_callback every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.get_logger().info('Turtle controller  has started!')
    
    def pose_callback(self,msg):
        """
        This function is called automatically whenever a message  is 
        received on /turtle1/pose.
        Args:
            msg (Pose): The received pose message containing x, y, theta
                        and  velocities
        """
        self.pose = msg

    def timer_callback(self):
        """
        This function is called every 0.5 seconds by the timer.
        It creates and publishes a Twist message to move the turtle.
        """

        if self.pose is None:
            return
        sensor_x = self.pose.x
        sensor_y = self.pose.y

        diff_x = self.goalx[self.side] - sensor_x
        diff_y = self.goaly[self.side] - sensor_y
        diff = math.sqrt(diff_x ** 2 + diff_y ** 2) 
        angle_diff = math.atan2(diff_y,diff_x) - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Create a new Twist message
        action = Twist()

        # Set linear velocity
        action.linear.x = self.kp_dist * diff
        # Set angular velocity
        action.angular.z = self.kp_ang * angle_diff

        # When it is near the corner, no linear velocity
        # Let the turtle turn the correct angle first
        if diff < self.threshold:
            self.side += 1
            action.linear.x = 0.0

            # Change the side index when finish a triangle
            if self.side >= 3:
                self.side = 0

        # Publish the message
        self.publisher_.publish(action)

        # Log the action
        self.get_logger().info(f'Publishing: linear.x={action.linear.x} angular.z={action.angular.z}')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our node
    turtle_controller = TurtleController()
    
    # Keep the node running and processing callbacks
    # This will run until you press Ctrl+C
    rclpy.spin(turtle_controller)
    
    # Cleanup
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
