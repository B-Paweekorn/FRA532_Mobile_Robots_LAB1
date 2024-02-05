import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf_transformations

# Define a Controller class that extends the Node class from ROS 2
class Controller(Node):
    def __init__(self):
        # Initialize the node with the name 'controller'
        super().__init__('controller')
        # Create a publisher for velocity commands
        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Set the timer period for periodic callback execution
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Subscribe to the pose topic to get the robot's current position
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # Initialize pose and goal
        self.pose = Pose()
        self.goal = np.array([0.0, 0.0])
        # Initialize TF2 buffer and listener for pose transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # Callback method for the timer
    def timer_callback(self):
        pass
    
    # Callback method for the pose subscription
    def pose_callback(self, msg):
        # Update the robot's current pose
        pass

    # Method to calculate the control commands
    def control(self):
        pass

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()