cmake_minimum_required(VERSION 2.8.3)
project(robotnik_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs std_msgs geometry_msgs message_generation)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    encoders.msg
    inputs_outputs.msg
    ptz.msg
    Data.msg
    Interfaces.msg
    Axis.msg
    AlarmSensor.msg
    Alarms.msg
    MotorStatus.msg
    MotorsStatus.msg
    State.msg
    BatteryStatus.msg
    BatteryStatusStamped.msg
    MotorHeadingOffset.msg
    MotorsStatusDifferential.msg
    InverterStatus.msg
    RobotnikMotorsStatus.msg
    ElevatorAction.msg
    ElevatorStatus.msg
    Cartesian_Euler_pose.msg
    alarmmonitor.msg
    alarmsmonitor.msg
    named_inputs_outputs.msg
    named_input_output.msg
    Register.msg
    Registers.msg
    StringArray.msg
    LaserMode.msg
    LaserStatus.msg
    SafetyModuleStatus.msg
    SubState.msg
    QueryAlarm.msg
    BatteryDockingStatus.msg
    BatteryDockingStatusStamped.msg
    BoolArray.msg
    MotorPID.msg
    Pose2DArray.msg
    Pose2DStamped.msg
    ReturnMessage.msg
)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    get_digital_input.srv
    set_analog_output.srv
    set_mode.srv
    set_ptz.srv
    get_mode.srv
    set_digital_output.srv
    set_odometry.srv
    set_height.srv
    enable_disable.srv
    home.srv
    axis_record.srv
    set_float_value.srv
    SetMotorStatus.srv
    SetElevator.srv
    get_alarms.srv
    ack_alarm.srv
    set_modbus_register.srv
    get_modbus_register.srv
    GetBool.srv
    set_CartesianEuler_pose.srv
    set_named_digital_output.srv
    SetLaserMode.srv
    ResetFromSubState.srv
    QueryAlarms.srv
    SetNamedDigitalOutput.srv
    InsertTask.srv
    SetMotorPID.srv
    SetMotorMode.srv
    SetBuzzer.srv
    SetEncoderTurns.srv
    GetMotorsHeadingOffset.srv
    SetByte.srv
)

add_action_files(
    FILES
    SetElevator.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
   actionlib_msgs
   std_msgs  # Or other packages containing msgs
   geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
  std_msgs
  geometry_msgs
)
