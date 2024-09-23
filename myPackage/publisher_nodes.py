#importing rclpy so its node class can be used
import rclpy 
from rclpy.node import Node

#importing the built-in array message type 
#that the node uses to structure the data that it passes on the topic
from std_msgs.msg import Int32MultiArray


#created a publisher_nodes class which inherits from Node (is a subclass of)
class publisher_nodes(Node):

    #defintion of the class constructor
    def __init__(self, node_name):
    
    	#calls the node Node class's constructor and gives it node name of 'minimal_publisher'
        super().__init__(node_name)
        
        
        #declares that node publishes messages of type Array over a topic name '/map' 
        #and that the queue size is 10
        self.map_publisher_ = self.create_publisher(Int32MultiArray, '/map', 10)
        self.goal_publisher_ = self.create_publisher(Int32MultiArray, '/goal', 10)
        self.start_publisher_ = self.create_publisher(Int32MultiArray, '/start', 10)
        
        self.timer = self.create_timer(1.0, self.publish_map)
        self.timer = self.create_timer(1.0, self.publish_goal_coordinate)
        self.timer = self.create_timer(1.0, self.publish_start_coordinate)
        
    #creates an map and publishes map to the console with get_logger().info
    def publish_map(self):
        my_map = [
        	[0, 1, 0, 0, 1, 1],
        	[0, 0, 1, 1, 0, 1],
        	[1, 0, 1, 1, 0, 1],
        	[0, 0, 1, 0, 0, 1],
        	[1, 0, 0, 0, 1, 1],
        	[1, 1, 1, 0, 0, 0],
        ]
        
        #flatten array
        flat_map = [item for sublist in my_map for item in sublist]

        
        msg = Int32MultiArray()
        msg.data = flat_map 
        self.map_publisher_.publish(msg)
        
        readable_map = '\n'.join([' '.join(map(str, row)) for row in my_map])
        
        self.get_logger().info(f'\nPublishing map:\n{readable_map}')

    def publish_goal_coordinate(self):
        my_goal_coordinate = [5, 5]
        msg = Int32MultiArray()
        msg.data = my_goal_coordinate
        self.goal_publisher_.publish(msg)
        self.get_logger().info(f'\nPublishing goal coordinate: {my_goal_coordinate}')   

    def publish_start_coordinate(self):
        my_start_coordinate = [0, 0]
        msg = Int32MultiArray()
        msg.data = my_start_coordinate
        self.start_publisher_.publish(msg)
        self.get_logger().info(f'\nPublishing start coordinate: {my_start_coordinate}')
        
#main function is defined
def main(args=None):
    #initializes the rclpy library
    rclpy.init(args=args)
    
    my_node_object = publisher_nodes('node_name')
    rclpy.spin(my_node_object)
    my_node_object.destroy_node()
    
    #node obj is created
    #map_publisher_object = publisher_nodes('map_node')
    #goal_coord_publisher_object = publisher_nodes('goal_node')
    #start_coord_publisher_object = publisher_nodes('start_node')

#call method
    #map_publisher_object.publish_map()
    #goal_coord_publisher_object.publish_goal_coordinate()
    #start_coord_publisher_object.publish_start_coordinate()
    
    #rclpy.spin(map_publisher_object)
    #rclpy.spin(goal_coord_publisher_object)
    #rclpy.spin(start_coord_publisher_object)

    # Destroy node and shutdown
    #map_publisher_object.destroy_node()
    #goal_coord_publisher_object.destroy_node()
    #start_coord_publisher_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
