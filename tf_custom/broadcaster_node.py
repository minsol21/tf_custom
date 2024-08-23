import rclpy
from rclpy.node import Node
from turtlebot4_msgs.msg import Turtlebot4Info  # import custom message
from geometry_msgs.msg import Pose
#from mocap4r2_msgs.msg import RigidBodies, RigidBody
import re

class InfoBroadcaster(Node):

    def __init__(self):
        super().__init__('info_broadcaster_node')

        # Parameter for simulation or real environment
        self.declare_parameter('environment', 'real')
        self.env = self.get_parameter('environment').get_parameter_value().string_value

        # Declare a 'role' parameter with 'follower' as the default value
        self.declare_parameter('role', 'follower')
        self.role = self.get_parameter('role').get_parameter_value().string_value

        # Get namespace
        self.namespace = self.get_namespace()

        # Initialize topics according to the namespace
        if self.namespace == '/':
            pose_topic = '/model/turtlebot4/pose'
            info_topic = '/tb_info_topic'
        else:
            pose_topic = f'/model{self.namespace}/turtlebot4/pose'
            info_topic = f'{self.namespace}/tb_info_topic'

        print(f"Namespace: {self.namespace}")
        print(f"Pose topic: {pose_topic}")
        print(f"Info topic: {info_topic}")

        if self.env == 'sim': # subscriber for simulation env
            self.subscriber = self.create_subscription(Pose, '/model' + self.namespace + '/turtlebot4/pose', self.sim_callback, 10) 
        
        #else: # subscriber for real env
        #    self.subscriber = self.create_subscription(RigidBodies, '/rigid_bodies', self.real_callback, 10)

        # publisher
        self.publisher = self.create_publisher(Turtlebot4Info, info_topic, 10)



    def sim_callback(self, pose_msg): # callback sim env

        info_msg = Turtlebot4Info()
        # check for a number in the namespace ans use as id
        if self.namespace == '/':
            info_msg.id = 0
        elif re.search(r'\d', self.namespace):
            info_msg.id = int(re.search(r'\d', self.namespace).group())
        else:
            info_msg.id = -1

        '''
        # check if the robot is the leader or follower in the namespace
        if 'leader' in self.namespace:
            info_msg.role = "leader"
        elif 'follower' in self.namespace:
            info_msg.role = "follower"
        else:
            info_msg.role = "leader" # TODO: change to a default role
        '''

        # Use the 'role' parameter value
        info_msg.role = self.role

        # Fill with the data coming from Ignition Gazebo
        info_msg.x = pose_msg.position.x
        info_msg.y = pose_msg.position.y
        info_msg.orientation.x = pose_msg.orientation.x
        info_msg.orientation.y = pose_msg.orientation.y
        info_msg.orientation.z = pose_msg.orientation.z
        info_msg.orientation.w = pose_msg.orientation.w

        self.publisher.publish(info_msg)

    def real_callback(self, rigid_bodies_msg): # callback real env

        info_msg = Turtlebot4Info()
        # check for a number in the namespace ans use as id
        if self.namespace == '/':
            info_msg.id = 0
        elif re.search(r'\d', self.namespace):
            info_msg.id = int(re.search(r'\d', self.namespace).group())
        else:
            info_msg.id = -1

        # check if the robot is the leader or follower in the namespace
        if 'leader' in self.namespace:
            info_msg.role = "leader"
        elif 'follower' in self.namespace:
            info_msg.role = "follower"
        else:
            info_msg.role = "leader" # TODO: change to a default role

        # fill with the data coming from Qualysis tracking system
        
        if self.namespace == '/':
            rigid_body = rigid_bodies_msg.rigidbodies[6]
        else:
            rigid_body = rigid_bodies_msg.rigidbodies[info_msg.id] # TODO how to understand which element of the array, can we use rigid body names for this?
                                                    # or we can assume that the first element is the reference frame, the second is the human
                                                    # and starting from the third will be a robot

        info_msg.x = rigid_body.pose.position.x
        info_msg.y = rigid_body.pose.position.y
        info_msg.orientation.x = rigid_body.pose.orientation.x
        info_msg.orientation.y = rigid_body.pose.orientation.y
        info_msg.orientation.z = rigid_body.pose.orientation.z
        info_msg.orientation.w = rigid_body.pose.orientation.w

        self.publisher.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    info_broadcaster_node = InfoBroadcaster()
    rclpy.spin(info_broadcaster_node)
    info_broadcaster_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
