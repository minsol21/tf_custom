import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math
import time

class TurtleBotMover(Node):
    def __init__(self, namespace):
        super().__init__('turtlebot_mover_' + namespace)

        # Declare parameters with default values
        self.declare_parameter('boundary_point1', (-3.0, -3.0))
        self.declare_parameter('boundary_point2', (3.0, 3.0))

        # Get parameters
        self.bpoint1 = self.get_parameter('boundary_point1').get_parameter_value().double_array_value
        self.bpoint2 = self.get_parameter('boundary_point2').get_parameter_value().double_array_value

        self.namespace = namespace
        self.publisher_ = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, f'/model/{namespace}/turtlebot4/pose', self.pose_callback, 10)
        self.current_position = None
        self.current_orientation = None

    def pose_callback(self, msg: Pose):
        self.current_position = (msg.position.x, msg.position.y)
        self.current_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def move_to_goal(self, goal_x, goal_y):
        print(f"Moving {self.namespace} to goal position: ({goal_x}, {goal_y})")

        while self.current_position is None:
            rclpy.spin_once(self)

        start_time = time.time()
        while True:
            current_x, current_y = self.current_position
            distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

            if time.time() - start_time >= 1:
                print(f"{self.namespace} current position: ({current_x}, {current_y}), Distance to goal: {distance:.2f}")
                start_time = time.time()

            if distance < 0.1:
                print(f"{self.namespace} has reached the goal position.")
                break

            self.rotate_towards_goal(goal_x, goal_y)

            msg = Twist()
            msg.linear.x = 0.2
            self.publisher_.publish(msg)
            rclpy.spin_once(self)
            time.sleep(0.2)

        self.stop_robot()

    def rotate_towards_goal(self, goal_x, goal_y):
        while True:
            current_x, current_y = self.current_position
            angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
            yaw = self.get_yaw()
            angle_diff = self.normalize_angle(angle_to_goal - yaw)

            print(f"{self.namespace} current yaw: {yaw:.2f}, Angle to goal: {angle_to_goal:.2f}, Angle diff: {angle_diff:.2f}")

            if abs(angle_diff) < 0.05:
                print(f"{self.namespace} is aligned with the goal direction.")
                break

            msg = Twist()
            msg.angular.z = 0.2 if angle_diff > 0 else -0.2
            self.publisher_.publish(msg)
            rclpy.spin_once(self)

        self.stop_robot()

    def get_yaw(self):
        x, y, z, w = self.current_orientation
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'{self.namespace} stopping the robot.')

def calculate_goal_positions(bpoint1, bpoint2, num_robots):
    x1, y1 = bpoint1
    x2, y2 = bpoint2
    distance_x = x2 - x1
    distance_y = y2 - y1
    interval_x = distance_x / (num_robots + 1)
    interval_y = distance_y / (num_robots + 1)

    # Calculate evenly spaced positions
    goals = []
    for i in range(num_robots):
        # Interpolate positions
        goal_x = x1 + (i + 1) * interval_x
        goal_y = y1 + (i + 1) * interval_y
        goals.append((goal_x, goal_y))

    return goals

def main(args=None):
    rclpy.init(args=args)

    namespaces = ['tb1', 'tb2', 'tb3']
    num_robots = len(namespaces)

    # Create nodes to get parameters
    nodes = []
    for ns in namespaces:
        node = TurtleBotMover(ns)
        nodes.append(node)

    # Fetch boundary points from the first node
    bpoint1 = nodes[0].bpoint1
    bpoint2 = nodes[0].bpoint2

    # Calculate goal positions for each robot
    goal_positions = calculate_goal_positions(bpoint1, bpoint2, num_robots)

    # Move robots to their respective goals
    movers = []
    for ns, goal in zip(namespaces, goal_positions):
        mover = TurtleBotMover(ns)
        movers.append(mover)
        mover.move_to_goal(*goal)

    for mover in movers:
        mover.destroy_node()

    for node in nodes:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
