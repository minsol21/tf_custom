# TurtleBot Line-Formation Simulation Setup

This guide provides step-by-step instructions to set up and drive multiple TurtleBots in a simulated environment using ROS 2 and Ignition Gazebo. The process includes environment setup, launching the simulation, bridging topics between Ignition and ROS 2, and controlling the TurtleBots.

## A. Environment Setup

1. **Open 8 Terminal Windows**  
   You will need to open 8 terminal windows for this setup, as each terminal will be used to control different aspects of the simulation.

2. **Build and Source the ROS 2 Workspace**
   Before proceeding, you'll need to clone the tf_custom repository into your ROS 2 workspace (ros2_ws). This package contains the necessary nodes for controlling the TurtleBots.

   Clone the `tf_custom` Repository
   
   Open a terminal and navigate to your ROS 2 workspace (`ros2_ws`):

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/minsol21/tf_custom.git
   ```
   
   This will download the `tf_custom` package into the `src` directory of your workspace.


   After cloning the repository, build and source your workspace to compile the newly added package.
   For each terminal, run the following commands to build and source your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
   

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## B. Simulation Setup

### 1. Launch TurtleBot Simulation with Namespaces

To simulate multiple TurtleBots, we will launch them in separate namespaces. This allows each TurtleBot to operate independently in the same simulation environment.

- **Launch TurtleBot 1 (tb1)**

   ```bash
   ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py namespace:=tb1 world:=empty
   ```

- **Launch TurtleBot 2 (tb2)**

   ```bash
   ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb2 x:=1
   ```

- **Launch TurtleBot 3 (tb3)**

   ```bash
   ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb3 x:=2
   ```

- **Launch Additional TurtleBot (tb4)**  
   For any additional TurtleBots, adjust the namespace and `x` coordinate:

   ```bash
   ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb4 x:=3
   ```

### 2. Set Up Ignition Topic Bridge

The Ignition topic bridge allows ROS 2 and Ignition Gazebo to communicate by bridging topics between them. 

1. **List Available Ignition Topics**  
   Run the following command to list available Ignition topics:

   ```bash
   ign topic -l 
   ```

2. **Bridge Pose Topics for Each TurtleBot**

   For each TurtleBot, run the corresponding command to bridge the `/pose` topic between Ignition and ROS 2:

   - **tb1:**

      ```bash
      ros2 run ros_gz_bridge parameter_bridge /model/tb1/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
      ```

   - **tb2:**

      ```bash
      ros2 run ros_gz_bridge parameter_bridge /model/tb2/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
      ```

   - **tb3:**

      ```bash
      ros2 run ros_gz_bridge parameter_bridge /model/tb3/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
      ```

   - **Additional TurtleBot (tb4):**

      ```bash
      ros2 run ros_gz_bridge parameter_bridge /model/tb4/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
      ```

3. **Verify the Bridge**  
   After bridging the topics, check that they are successfully bridged by listing ROS 2 topics:

   ```bash
   ros2 topic list
   ```

   You should see topics like `/model/tb1/turtlebot4/pose`, `/model/tb2/turtlebot4/pose`, etc.

   To verify, echo the pose of one of the TurtleBots:

   ```bash
   ros2 topic echo /model/tb1/turtlebot4/pose
   ```

### 3. Drive TurtleBot

With the topics bridged, you can now control the TurtleBots. 

1. **Run the TurtleBot Mover Node**  
   Use the `turtlebotmover` node to position the TurtleBots along a line between two points. You can declare the boundary points as parameters via the command line:

   ```bash
   ros2 run tf_custom turtlebotmover --ros-args -p boundary_point1:="[ -2.0, -2.0 ]" -p boundary_point2:="[ 2.0, 2.0 ]"
   ```

   Another example:

   ```bash
   ros2 run tf_custom turtlebotmover --ros-args -p boundary_point1:="[ 0.0, -2.0 ]" -p boundary_point2:="[ 4.0, -2.0 ]"
   ```

   - `boundary_point1` and `boundary_point2` define the line along which the TurtleBots will be positioned.

![TurtleBot Movement Visualization](https://prod-files-secure.s3.us-west-2.amazonaws.com/ba8e00c2-fec6-4186-ac41-8f215ba31d9b/2f45dc2e-29e6-4423-8fc5-eea5c55528a0/image.png)

**Node:** `turtlebotmover.py`

---

This version of the README is more structured, with added explanations and clarifications for each step. This should make it easier for users to follow the setup process and understand the commands they are executing.
## A. Environment Setup

open the default 8 terminal

for every terminal, `colcon build` and `source install`

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## B. Simulation Setup

### 1. Namespace turtlebots

for each terminal, run those command.

tb1

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py namespace:=tb1 world:=empty
```

tb2

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb2 x:=1
```

tb3

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb3 x:=2
```

for additional turtlebot, 

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=tb4 x:=3
```

### 2. Ignition topic bridge

```bash
ign topic -l 
```


**tb1**

```bash
ros2 run ros_gz_bridge parameter_bridge /model/tb1/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
```

**tb2**

```bash
ros2 run ros_gz_bridge parameter_bridge /model/tb2/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose

```

**tb3**

```bash
ros2 run ros_gz_bridge parameter_bridge /model/tb3/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
```

for additional turtlebot, 

```bash
ros2 run ros_gz_bridge parameter_bridge /model/tb4/turtlebot4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose

```

now, check with `ros2 topic list` if it’s bridged well

```bash
ros2 topic list
```

then you’ll find these 4 pose topics in ros2 now.

`/model/tb1/turtlebot4/pose`

`/model/tb2/turtlebot4/pose`

`/model/tb3/turtlebot4/pose`

![Untitled](https://prod-files-secure.s3.us-west-2.amazonaws.com/ba8e00c2-fec6-4186-ac41-8f215ba31d9b/5b03f564-ffcb-41dd-9aa9-23c8bc49e0a5/Untitled.png)

check with `ros2 topic echo`

```jsx
ros2 topic echo /model/tb1/turtlebot4/pose
```

![Untitled](https://prod-files-secure.s3.us-west-2.amazonaws.com/ba8e00c2-fec6-4186-ac41-8f215ba31d9b/ce915868-6a14-42fd-a920-eac8ec803031/Untitled.png)

### 3. Drive Turtlebot

[turtlebotmover.py](https://prod-files-secure.s3.us-west-2.amazonaws.com/ba8e00c2-fec6-4186-ac41-8f215ba31d9b/28724728-f34c-4e22-afb2-893b3e2beb98/turtlebotmover.py)

Now, using these topics, we can drive turtlebot with the node `turtlebotmover` 

we can position our turtlebots along the line evenly between two points

we can declare parameters with the command line

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run tf_custom turtlebotmover --ros-args -p boundary_point1:="[ -2.0, -2.0 ]" -p boundary_point2:="[ 2.0, 2.0 ]"
```

```bash
ros2 run tf_custom turtlebotmover --ros-args -p boundary_point1:="[0.0, -2.0 ]" -p boundary_point2:="[ 4.0, -2.0 ]"
```

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/ba8e00c2-fec6-4186-ac41-8f215ba31d9b/2f45dc2e-29e6-4423-8fc5-eea5c55528a0/image.png)
