## Disclaimer  , this is all from opensource projects 

 
 I find the lack of a ackerman robot with properly documented instructions on how to get it working , thus i am taking the time to do it .

Hope you find this helpful!

![](docs/coordinates_wheels.jpg) 
 
 
##### List of original repositories i have downloaded the code from
 --------------------------------------
  

  
- [https://github.com/RobotnikAutomation/rbcar_sim](https://github.com/RobotnikAutomation/rbcar_sim) 

- [https://github.com/RobotnikAutomation/rbcar_common](https://github.com/RobotnikAutomation/rbcar_common) 

- [https://github.com/RobotnikAutomation/robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs) 

- [https://github.com/RobotnikAutomation/robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors) 


- [https://github.com/gkouros/ackermann-drive-teleop](https://github.com/gkouros/ackermann-drive-teleop) 



#### Pre requisites

-  install gazebo 9+
-  works well on ros kinetic


Install all the other required packages

		sudo apt install 


### How to run the gazebo sim
-----------------------------------------------------

This will launch the sonoma_raceway by default .

	roslaunch rbcar_sim_bringup rbcar_complete.launch

Once Gazebo is launched succefully 
	
	roslaunch rbcar_control rbcar_control.launch	

![](docs/gazebo.gif) 



To control the robot with **/cmd_vel**


	roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch



To launch gmapping and move_base


	roslaunch rbcar_localization slam_gmapping.launch


To launch **move_base with TEB planner**

	roslaunch teb_local_planner_tutorials robot_carlike_in_stage.launch
	

To launch **RVIZ**

	roslaunch rbcar_viz view_robot.launch

![](docs/rviz.gif) 

