#!/usr/bin/env python


import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys

class AckermannDriveJoyop:

    def __init__(self, args):
        if len(args)==1 or len(args)==2:
            self.max_speed = float(args[0])
            self.max_steering_angle = float(args[len(args)-1])
            cmd_topic = 'ackermann_cmd'
        elif len(args) == 3:
            self.max_speed = float(args[0])
            self.max_steering_angle = float(args[1])
            cmd_topic = '/' + args[2]
        else:
            self.max_speed = 2
            self.max_steering_angle = 0.7
            cmd_topic = '/rbcar_robot_control/command'

        self.speed = 0
        self.steering_angle = 0
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.drive_pub = rospy.Publisher(cmd_topic, AckermannDriveStamped,
                                         queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        rospy.loginfo('ackermann_drive_joyop_node initialized')

    def joy_callback(self, joy_msg):
        self.speed = self.map(1,-1,0,1,joy_msg.axes[5] )* self.max_speed;
        if(joy_msg.buttons[4]):
            self.speed = -self.map(1,-1,0,1,joy_msg.axes[5] )* self.max_speed;
        self.steering_angle = self.map(-1,1,-1,1,joy_msg.axes[0] * self.max_steering_angle);

    def cmd_vel_callback(self, cmd_vel_msg):
        self.speed = cmd_vel_msg.linear.x* self.max_speed;
        self.steering_angle = cmd_vel_msg.angular.z * self.max_steering_angle;
    
    
    def map(self,in_min, in_max, out_min, out_max,x):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        


    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp = rospy.Time.now()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(ackermann_cmd_msg)
        self.print_state()

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp = rospy.Time.now()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.drive_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])
    rospy.spin()
