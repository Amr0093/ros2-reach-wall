# 🦾 ROS2 Reach Wall Project

This repository is an **educational ROS 2 workspace** built for learning and experimenting with fundamental ROS2 concepts:
- **Topics**
- **Services**
- **Actions**
- **Integration with TurtleBot3 and Gazebo**

The workspace demonstrates how to create custom service and action interfaces, build Python nodes that interact with them, and test them in simulation.

____________________________________________________________________________________________________________________________________________________________________________________________________________________

## 📁 Project Structure

colcon1_ws/
├── add_two_ints_srv/ # Python package for a custom service (AddTwoInts)
│ ├── add_two_ints_srv/
│ │ ├── client.py # Client node to request sum of two integers
│ │ ├── server.py # Service server that processes requests
│ │ └── init.py
│ ├── resource/
│ │ └── add_two_ints_srv
│ ├── setup.py
│ ├── setup.cfg
│ ├── package.xml
│ └── test/
│
├── src/
│ ├── my_own_service/ # Custom .srv definition (AddTwoInt.srv)
│ ├── reach_wall_action/ # Custom .action definition (ReachWall.action)
│ ├── move_robot/ # Python action server/client nodes using the action
│ ├── my_first_package/ # Simple pub/sub examples
│ └── turtlebot3_custom_pub/ # Publisher to control TurtleBot3 in Gazebo
│
└── .gitignore


____________________________________________________________________________________________________________________________________________________________________________________________________________________

## ⚙️ Requirements

- **ROS2 Foxy** or later  
- **colcon** build tool  
- **TurtleBot3 packages**  
- **Gazebo simulation environment**
____________________________________________________________________________________________________________________________________________________________________________________________________________________
Install missing dependencies if needed:
```bash
sudo apt install ros-foxy-turtlebot3* ros-foxy-gazebo-ros-pkgs
🧱 Building the Workspace
From the root of the workspace (colcon1_ws):

bash
colcon build --symlink-install
source install/setup.bash
If you want to build a single package only (to avoid build errors from others):

bash
colcon build --packages-select move_robot
🧩 Package Summaries
1. add_two_ints_srv
A simple service package demonstrating request-response communication.

Service definition: my_own_service/srv/AddTwoInt.srv

Server: Waits for two integers, returns their sum.

Client: Sends two integers, logs the result.
___________________________________________________________________________________________________________________________________________________________________________________________________________
Example usage:

bash
ros2 run add_two_ints_srv server
ros2 run add_two_ints_srv client 3 7
2. reach_wall_action
Defines a custom action interface used to make a robot move forward until it reaches a wall.

Action file: ReachWall.action


# Goal
float32 target_distance
---
# Result
bool reached
---
# Feedback
float32 remaining_distance
3. move_robot
Implements an Action Server and Action Client in Python.

The server subscribes to /scan, publishes /cmd_vel, and moves the robot forward until the target_distance is reached.

The client sends a goal and receives continuous feedback on the remaining distance.

Run the server and client:

bash
# Terminal 1 - Start Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py

# Terminal 2 - Run action server
ros2 run move_robot action_server

# Terminal 3 - Send goal
ros2 action send_goal /reach_wall reach_wall_action/action/ReachWall "{target_distance: 0.5}" --feedback
If you want to reset the simulation:

bash
ros2 service call /reset_world std_srvs/srv/Empty {}
🐢 Simulation Integration
You can visualize everything in Gazebo using TurtleBot3:

bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
Active topics:

bash
ros2 topic list
Expected key topics:

/cmd_vel – Velocity commands to robot

/scan – LIDAR data

/odom – Odometry info
____________________________________________________________________________________________________________________________________________________________________________________________________________________
💡 Tips & Troubleshooting
Always source install/setup.bash before running nodes.

To rebuild cleanly:

bash
rm -rf build install log
colcon build --packages-select <package_name>
Check interface definitions:

bash
ros2 interface show reach_wall_action/action/ReachWall
If the robot isn’t moving, ensure:

/cmd_vel topic exists.

Gazebo simulation is running.

You’ve launched the correct TurtleBot3 model.
____________________________________________________________________________________________________________________________________________________________________________________________________________________
📘 Educational Goal
This project is intended for anyone learning ROS2 fundamentals.
It builds understanding step by step:

Start with simple publishers/subscribers.

Move to custom services (AddTwoInts).

Progress to actions (ReachWall) and robot simulation.

🧑‍💻 Author
Amr Hassan
📧 amr.hassan0528@gmail.com
🔗 github.com/Amr0093

🏁 License
This repository is for educational purposes.
Feel free to use, modify, and share it to learn ROS2.
