import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

# Create a node that will publish the joint states 
# References: 
# Python library: https://docs.python.org/3/library/functions.html 
# ROS2 Humble documentation: 
# RVIZ User Guide: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html

# 
class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_limits_node')
        
        self.progress = 0
        
        self.user_input()
                
        self.move_duration = 10 # How long the move should take
        
        # Create publisher on the joint_states topic
        self.publisher_ = self.create_publisher(JointState, 'joint_states',10)
        
        # Define clock parameters
        UPS = 60 # How many times the joint angles should be updated per second
        timer_period = 1 / UPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Get start time
        self.start_time = self.get_clock().now()
        
        # Create log
        # self.get_logger().info('Move publisher started.')
        # self.get_logger().info(f'Moving J1: {self.j1_start} -> {self.j1_end} deg')
        # self.get_logger().info(f'Moving J2: {self.j2_start} -> {self.j2_end} deg')


    def user_input(self):
        
        if self.progress >= 1.0:
            self.deg1_start = self.deg1_end
            self.deg2_start = self.deg2_end
            self.deg1_end = float(input("Input next position for joint 1 "))
            self.deg2_end = float(input("Input next position for joint 2: "))
        else:
            self.deg1_start = float(input("Input starting angle for joint 1: "))
            self.deg1_end = float(input("Input ending angle for joint 1: "))
            self.deg2_start = float(input("Input starting angle for joint 2: "))
            self.deg2_end = float(input("Input ending angle for joint 2: "))
    
        # Define the starting angles
        self.j1_start = math.radians(self.deg1_start) 
        self.j2_start = math.radians(self.deg2_start)
        
        # Define the end angles
        self.j1_end = math.radians(self.deg1_end)
        self.j2_end = math.radians(self.deg2_end)
        
        # Find the range of the joints
        self.j1_range = self.j1_end - self.j1_start
        self.j2_range = self.j2_end - self.j2_start
        
        # Create log
        # self.get_logger().info('Move publisher started.')
        # self.get_logger().info(f'Moving J1: {self.j1_start} -> {self.j1_end} deg')
        # self.get_logger().info(f'Moving J2: {self.j2_start} -> {self.j2_end} deg')
            
    def timer_callback(self):
        
        # Get current time
        current_time = self.get_clock().now()
        
        # Get elapsed time
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        # Calculate progress from 0.0 to 1.0
        # This will be used to smoothly move the joints over chosen duration
        # (there may be a better way to do this, but this is the best way I can think of)
        self.progress = min(elapsed_time / self.move_duration, 1.0) # this uses min so it cannot go over 1.0
        
        # Animation Logic
        
        theta1 = self.j1_start + (self.j1_range * self.progress)
        theta2 = self.j2_start + (self.j2_range * self.progress)
        
        # Create the message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder','elbow']
        msg.position = [theta1,theta2]
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Stop if its done
        if self.progress >= 1.0:
            self.get_logger().info('Target Position reached')
            self.user_input()
            self.timer.reset()
            self.start_time = self.get_clock().now()

            
def main(args=None):
    rclpy.init(args=args)
    
    joint_limits_node = JointPublisher()
    
    try:
        rclpy.spin(joint_limits_node)
    except KeyboardInterrupt:
        pass
    
    joint_limits_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()