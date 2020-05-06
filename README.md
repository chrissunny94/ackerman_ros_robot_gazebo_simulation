## Disclaimer  , this is all from opensource projects 

 
 I find the lack of a ackerman robot with properly documented instructions on how to get it working , thus i am taking the time to do it .

Hope you find this helpful!

![](rbcar_common/rbcar_description/meshes/coordinates_wheels.jpg) 
 
 
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
	
	

![](docs/gazebo.gif) 



