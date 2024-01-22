## Robot Package Template

Packages needed:
	joint_state_publisher_gui
	slam_toolbox
	rplidar_ros
	gazebo
	
Spawning robot in gazebo: roslaunch my_bot spawn.launch

Launching Joint State Publisher: rosrun joint_state_publisher_gui joint_state_publisher_gui
Launching a world file: gazebo /home/william/catkin_ws/src/my_bot/worlds/tracked_vehicle_simple.world
	Maybe: roslaunch gazebo_ros empty_world.launch

Keyboard Control:  rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Slam: roslaunch slam_toolbox online_async.launch params_file:=/home/william/catkin_ws/src/my_bot/config/mapper_params_online_async.yaml