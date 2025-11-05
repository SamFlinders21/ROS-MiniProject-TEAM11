import math 
import rclpy
from rclpy.node import Node

# These message types are required
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class FkCalculator(Node):
    
    def __init__(self):
        super().__init__('fk_node')
        
        # Define each link length
        
        self.L1 = 1.0
        self.L2 = 1.0
        
        # Creates a subscriber node that listens to the joint_states topic, 
        # expecting a "JointState" message. Each time that it receives one, 
        # it calls the self.joint_callback function
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Make a publisher that will publish on the /ee_position topic
        self.position_publisher_ = self.create_publisher(Point,'ee_position', 10)
        
        # Make a node that publish a Marker message for RViz
        self.marker_publisher_ = self.create_publisher(Marker, 'arm_marker',10)
        
    # Create the callback function mentioned earlier
    # 'msg' is the JointState message data
    # 
    def joint_callback(self, msg):
        
        rad1 = msg.position[0]
        rad2 = msg.position[1]
    
        # Calculate link 1 positions
        x1 = self.L1 * math.cos(rad1)
        y1 = self.L1 * math.sin(rad1)
    
        # Calculate link 2 positions
        x2 = x1 + self.L2 * math.cos(rad1 + rad2)
        y2 = y1 + self.L2 * math.sin(rad1 + rad2)
        
        # Converting into degrees to make the terminal more readable
        deg1 = math.degrees(rad1)
        deg2 = math.degrees(rad2)
        
        # Print the result to the terminal
        self.get_logger().info(f"Angles: [t1={deg1:.2f}, t2={deg2:.2f}] End Effector Position: [x={x2:.2f}, y={x2:.2f}]")
        
        # Publish the result
        output_msg = Point()
        output_msg.x = x2
        output_msg.y = y2
        output_msg.z = 0.0
        
        self.position_publisher_.publish(output_msg)
        
        self.publish_arm_marker(x1,y1,x2,y2)
        
    # Create function to create and publish the arm marker
    def publish_arm_marker(self,x1,y1,x2,y2):
        marker = Marker()
        marker.header.frame_id = "world" # Sets the coordinate frame to "world" (RViz will look wrong without this line)
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 0 # This gives the marker an ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set line width
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        
        # Set line color (Blue)
        #marker.color.r = 0.0
        #marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        p_origin = Point() # Defines the origin of link 1
        
        # Define position of the elbow (the point where l1 and l2 connect)
        p_elbow = Point()
        p_elbow.x = x1
        p_elbow.y = y1
        
        # Define position of the End Effector (hand)
        p_hand = Point()
        p_hand.x = x2
        p_hand.y = y2
        
        # Add the points to the marker's list    
        marker.points = [p_origin, p_elbow, p_hand] 
        
        self.marker_publisher_.publish(marker)
       
def main(args=None):
    rclpy.init(args=args)
    
    fk_node = FkCalculator()
    
    # make it so the node loops until we exit it
    try:
        rclpy.spin(fk_node)
    except KeyboardInterrupt:
        pass
    
    fk_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()