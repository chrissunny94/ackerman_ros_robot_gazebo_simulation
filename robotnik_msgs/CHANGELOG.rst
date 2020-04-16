^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotnik_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.0.0 (2019-11-05)
------------------
* Battery current removed from docking status msg
* Add new safety mode for SafetyModuleStatus
* Add current to BatteryStatus msg
* UpdateMotorHeadingOffset msg & srv
* Add service to set a byte
* Added Motor heading offset and turns msgs
* SetBuzzer srv added
* Merged from upstream
* Added package reference to prevent msg collisions and height param to ElevatorStatus
* Add new msg Pose2DStamped
* solved building issue
* added warning zones to laser
* added messages to set motor pid
* added speed to SafetyModuleStatus
* added invalid mode to LaserMode
* Merging from upstream
* Add message Pose2DArray
* Added new srv InsertTask

0.2.5 (2019-01-24 11:37:54 +0100)
---------------------------------
* add srv and msg to query alarms
* Create Register msg for Modbus communication
* Add SetNamedDigitalOutput srv
  This message is the same than set_named_digital_output following
  the right naming
* Adding message for an array of Booleans
* Adding flag lasers_on_standby into SafetyModuleStatus
* Adding BatteryDockingStatus stamped
* setting format of battery status stamped in the correct way
* Adding new messages for battery docking status and battery status stamped
* updated safety msgs
* Added srv ResetFromSubState
* added SubState.msg
* correcting changelog format
* undone last commit: zones msgs are now in their own package
  btw, with previous commit package did not build
* Added robotnik_navigation_tools msgs
* Removed lasers ok field
* Update LaserStatus.msg
* Update SafetyModuleStatus.msg
* Added new fields to SafetyModuleStatus

0.2.4 (2018-07-16 17:19:51 +0200)
---------------------------------
* updated changelog
* LaserStatus: added free_warning flag
* SafetyModuleStatus msg: added manual_realeas and bumper_override
* Added SafetyModuleStatus and LaserStatus msg. Added SetLaserMode service
* added time_charging to BatteryStatus.msg
* added is_charging flag to BatteryStatus.msg
* added averagecurrent and analog inputs to MotorStatus msg

0.2.3 (2018-05-14 12:57:16 +0200)
---------------------------------
* formatting changelog msgs
* updating changelog
* adding mantaineirs for the package
* added SetElevator action
* robotnik_msgs: completing alarms
* removed set_kuka_pose.srv
* Cartesian euler pose msg and srv added
* robotnik_msgs: adding string array
* Kuka pose msg and set kuka pose srv added
* Kuka pose msg and set kuka pose srv added
* added set_named_input to CMakeLists
* msg changed to digital_inputs and digital_outputs
* robotnik_msgs: alarms with display number
* Added named_input_output msg and srv
* added GetBool service

0.2.2 (2018-02-16 13:02:07 +0100)
---------------------------------
* added list of strings of active status word and flags
* added set/get modbus register message
* robotnik_msgs: alarms msgs
* adding new msgs and srvs for a Elevator system
* adding voltage to BatteryStatus.msg
* Merge branch 'kinetic-multi-devel'
* merging with kinetic-devel
* Adding new msg for robotnik_base_hw
* robotnik_msgs: Adding I/O to motor status
* renamed InverterState.msg to InverterStatus.msg
* added InverterState message

0.2.1 (2016-07-12 07:15:59 +0200)
---------------------------------
* updated changelog
* added MotorsStatusDifferential.msg
* added BatteryStatus msg
* Contributors: carlos3dx, rguzman

0.2.0 (2015-07-17 10:25:30 +0200)
---------------------------------
* Editing changelog
* Setting version 0.2.0
* Adding new field for the Axis.msg
* Adding msg State.msg
* Adding new msg State

0.1.0 (2014-08-06)
------------------
* Adding CHANGELOG and gitignore files
* Adding new service set_float_value.srv
* Fixing dependencies problems
* Adding initial list of messages and services
* Initial commit
