# ackermann_drive_teleop
ROS teleoperation scripts for robots with ackermann steering

##### ackermann_drive_keyop
+ Run the teleoperation script, with  
`rosrun ackermann_drive_teleop keyop.py`  
+ You can set max speed, steering angle and command topic by giving them as arguments, when running the script.  
eg.1 `rosrun ackermann_drive_teleop keyop.py 0.5`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; -> max_speed=max_steering_angle=0.5, command_topic=/ackermann_cmd  
eg.2 `rosrun ackermann_drive_teleop keyop.py 0.5 0.8`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ->  max_speed=0.5, max_steering_angle=0.8, command_topic=/ackermann_cmd  
eg.3 `rosrun ackermann_drive_teleop keyop.py 0.5 0.8 ack_cmd`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ->  max=speed=0.5, max_steering_angle=0.8, command_topic=/ack_cmd  
+ Use the "up", "down" arrow keys to control speed, "left" and "right" arrow keys to control the steering angle,  
  space to brake and tab to reset the steering angle.  

##### ackermann_drive_joyop
+ Run the teleoperation script, as well as the joy node using the following command:  
`roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch`  
+ You can set max speed and steering angle, by giving them as arguments to the launcher.  
eg. `roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch max_speed:=0.5 max_angle:=0.8`  
+ **In order to use a joystick, it must have read and write permissions.**  
You can grant such permissions by executing the following command: `sudo chmod a+rw /dev/input/js0`
