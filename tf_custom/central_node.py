import rclpy
from rclpy.node import Node
from turtlebot4_msgs.msg import Turtlebot4Info
from geometry_msgs.msg import Pose
import math

class CentralNode(Node):

    def __init__(self):
        super().__init__('central_node')
        
        # Declare subscriptions for each TurtleBot's pose
        self.turtlebot_names = ['/tb1', '/tb2', '/tb3', '/tb4']
        self.poses = {}
        self.subscribers = {}
        
        for name in self.turtlebot_names:
            pose_topic = f'{name}/turtlebot4/pose'
            self.subscribers[name] = self.create_subscription(
                Pose,
                pose_topic,
                lambda msg, name=name: self.pose_callback(msg, name),
                10)
        
        self.publishers = {}
        for name in self.turtlebot_names:
            self.publishers[name] = self.create_publisher(
                Turtlebot4Info,
                f'{name}/tb_info_topic',
                10)
        
    def pose_callback(self, msg, name):
        self.poses[name] = msg
        self.calculate_distances()
        
    def calculate_distances(self):
        if len(self.poses) < len(self.turtlebot_names):
            return  # Wait until all positions are received
        
        distances = {}
        for i, name1 in enumerate(self.turtlebot_names):
            for name2 in self.turtlebot_names[i+1:]:
                pos1 = self.poses[name1]
                pos2 = self.poses[name2]
                dist = self.compute_distance(pos1, pos2)
                distances[(name1, name2)] = dist
                distances[(name2, name1)] = dist
        
        self.publish_distances(distances)
    
    def compute_distance(self, pos1, pos2):
        dx = pos1.position.x - pos2.position.x
        dy = pos1.position.y - pos2.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def publish_distances(self, distances):
        for name in self.turtlebot_names:
            info_msg = Turtlebot4Info()
            info_msg.id = int(name[3])  # Extracting robot number from '/tbX'
            info_msg.role = "info"  # Or some role for info node
            
            # Collect relevant distances for this TurtleBot
            relevant_distances = {k: v for k, v in distances.items() if k[0] == name}
            
            # Publish distances
            for (src, dst), dist in relevant_distances.items():
                # Here you can customize the message to include distances
                # For simplicity, let's assume a simple format
                info_msg.x = dist  # Use `x` field to store distance
                self.publishers[name].publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    central_node = CentralNode()
    rclpy.spin(central_node)
    central_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
