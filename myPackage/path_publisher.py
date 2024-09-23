#importing useful package that has class Node
import rclpy
from rclpy.node import Node

#alows for arrays to be stored as a message (an integer array)
from std_msgs.msg import Int32MultiArray

#new class that inherits from Node
class path_publisher(Node):

    #calling Node class constructor and initialzing a node w/ a name
    def __init__(self):
        super().__init__('path_node')
        
        #creating a publisher that sends an array message that goes through a topic called path (10 represents queue size)
        self.publisher = self.create_publisher(Int32MultiArray, '/path', 10)
        
        #creating 3 subscribers that come through the 3 topics we made before and calls the callback methods once recieved
        self.map_subscriber = self.create_subscription(Int32MultiArray, '/map', self.map_callback, 10)
        self.goal_subscriber = self.create_subscription(Int32MultiArray, '/goal', self.goal_callback, 10)
        self.start_subscriber = self.create_subscription(Int32MultiArray, '/start', self.start_callback, 10)
        
        #self.subscriptions #prevent unused variable warning
        
        #initializing variables to hold the received data
        self.received_map = None
        self.received_start = None
        self.received_goal = None
        
        #creating flags that allows us to know if start and goal have been processed --> so we can find path w/ all the info
        self.start_processed = False
        self.goal_processed = False
        #self.timer = self.create_timer(1.0, self.publish_path)

    #the next three methods are called when a new map, start position, and end position is received
    #they store received data the variables we declared in constuctor
    #finally they call the check_before_path() to check if we have all the details
    #Note: msg is incoming message, msg.data retrieves the actual data
    def start_callback(self, msg):
        #storing in a tuple
    	self.received_start = (msg.data[0], msg.data[1])
    	self.get_logger().info(f'\nReceived start: {self.received_start}')
    	self.start_processed = True
    	self.check_before_path()

    def goal_callback(self, msg):
    	self.received_goal = (msg.data[0], msg.data[1])
    	self.get_logger().info(f'\nReceived goal: {self.received_goal}')
    	self.goal_processed = True
    	self.check_before_path()

    def map_callback(self, msg):
    	#read the map and convert it back to a 2D array
    	
    	#initializes an empty array ready to store the map
    	self.received_map = []
    	#find size of map by taking sqrt of it and turning it into integer
    	num_of_elements = len(msg.data)
    	#this gives you the # of rows and the # of elements in each row
    	map_size = int(num_of_elements ** 0.5)  # Assuming the map is square
    	
    	#for each row, append the next set of elements
    	#self.received_map = [list(msg.data[i*map_size:(i+1)*map_size]) for i in range(map_size)]
    	for i in range(map_size):
    	    start_index = i*map_size
    	    end_index =  (i+1)*map_size
    	    #append a section (one row) from flat list at a time to get a 2D array
    	    self.received_map.append(msg.data[start_index: end_index])
    	
    	self.get_logger().info(f'\nReceived map: {self.received_map}')
    	
    	#call a method that computes the path
    	self.check_before_path()


    #to avoid creating a path w/o all info recieved
    def check_before_path(self):
        if self.received_start is not None and self.received_goal is not None:
            self.find_path()
            
    #get priority of a certain coordinate and if it doesn't have a priority, give it infinity        
    #def get_priority(self, coordinate, priority):
    	#return priority.get(coordinate, float('inf'))
    
    #the A* algorithm: consider coordinates until reached goal, then retrace steps
    def find_path(self):
        start = self.received_start
        goal = self.received_goal
        
        #our set of coordinates that we are considering (like our priority queue)
        considering_set = {start}
        #store necessary info we need abt each coordinate
        prev_ref = {}
        cost = {start: 0}
        priority = {start: self.distance_left(start, goal)}
        
        #where we consider the top coordinate on our priority queue
        while considering_set:
            #find the smallest one in the set, use priority to compare it
            #current = min(considering_set, key=get_priority(x, priority))
            current = min(considering_set, key=lambda x: priority.get(x, float('inf')))
            #base case: if we reached our goal stop and make path
            if current==goal:
                self.solidify_path(current, prev_ref)
                return
        
        #once done considering, remove it and find possible next steps
            considering_set.remove(current)
            surrounding = self.get_surrounding(current)
        
        #for each of those possible next steps
            for x in surrounding:
            #if this is the shortest path to new coordinate and its not an obstacle
                if(cost[current]+1<cost.get(x, float('inf'))) and (self.is_clear(x)):
                #update info & calc priority & add it to the set to consider
                    cost[x] = cost[current] + 1
                    prev_ref[x] = current
                    priority[x] = cost[x] + self.distance_left(x, goal)
                    considering_set.add(x)
    
    #checks if there is an obstacle or not and on the map
    def is_clear(self, coordinate):
        x, y, = coordinate
        return(0<=x<len(self.received_map) and
            0<=y<len(self.received_map[0]) and
            self.received_map[x][y]==0)
            
    def get_surrounding(self, coordinate):
        x, y = coordinate
        return [
            (x - 1, y),  # Up
            (x + 1, y),  # Down
            (x, y - 1),  # Left
            (x, y + 1)   # Right
        ]
    
    #once we have the last coordinate, back track and store the path
    def solidify_path(self, current, prev_ref):
        path = []
        #while we have the previous reference
        while current in prev_ref:
            #append it and go to previous
            path.append(current)
            current = prev_ref[current]
        #reverse list so it's in the correct order
        path.reverse()
        self.publish_path(path)


    def distance_left(self, current, goal):
    
        #since you can only move one unit up/down/left/right, from each coordinate you must go at least a certain amount down, and a certain amount to the right
        return (goal[0]-current[0]) + (goal[1]-current[1]) 
    

    def publish_path(self, my_path):
        #expected path:
        #my_path = [
        #    [0, 0],
        #    [1, 0],
        #    [1, 1],
        #    [2, 1],
        #    [3, 1],
        #    [4, 1],
        #    [4, 2],
        #    [4, 3],
        #    [5, 3],
        #    [5, 4],
        #]
        
        #flattens the path coordinates and publishes it as msg
        flatten_path = [coord for point in my_path for coord in point]
        
        msg = Int32MultiArray()
        msg.data = flatten_path
        
        self.publisher.publish(msg)
        #prints to terminal
        self.get_logger().info(f'\nPublishing path: {my_path}')
        
        

def main(args=None):
    #initializing ros2 communication
    rclpy.init(args=args)

    #creating instance which uses ros2 communication
    path_publishing_object = path_publisher()
    rclpy.spin(path_publishing_object)

    #destory node when done
    path_publishing_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
