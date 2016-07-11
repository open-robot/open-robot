#!/usr/bin/env python

import thread
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseWithCovarianceStamped
twist = Twist()

pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

joystick_ready = False
lock_v = False
def initial_pose_reset():
    pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x=0
    pose.pose.pose.position.y=0
    pose.pose.pose.position.z=0
    pose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    pose.pose.pose.orientation.z=0.0267568523876
    pose.pose.pose.orientation.w=0.999641971333
    rospy.loginfo(pose)
    pose_pub.publish(pose)
    #rospy.spin()


def cmd_vel_puber():
    global twist
    global joystick_ready
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if joystick_ready:
            pub.publish(twist)
            print "x: %s, y: %s, za: %s" %(twist.linear.x,
                                        twist.linear.y,
                                        twist.angular.z)
        rate.sleep()

def joy_cb(joy_data):
    global joystick_ready
    global twist
    print joy_data
    global lock_v
    # start with menu button
    if joy_data.buttons[11] == 1:
        joystick_ready = not joystick_ready

    # reset to home with return button
    if joy_data.buttons[10] == 1:
        initial_pose_reset()

    if not joystick_ready:
        return


    if joy_data.buttons[0] == 1:
         lock_v = not lock_v

    if lock_v :
        print "speed locked"
        y_axes = 0.0
        x_axes = 0.2
    else:
        print "speed unlocked"
        x_axes = joy_data.axes[1]*0.8
        y_axes = joy_data.axes[0]*0.8

    if joy_data.axes[6] != 1:
        z = (joy_data.axes[6] -1  ) *2
    else:
        z = (1-joy_data.axes[7]) *2

    twist.linear.x = x_axes
    twist.linear.y = y_axes
    twist.angular.z = z
    pub.publish(twist)    

def subscriber():



    rospy.Subscriber("joy", Joy, joy_cb)

    rospy.spin()
    
if  __name__ == '__main__':
    rospy.init_node("joystick", anonymous=True)    
    thread.start_new_thread(cmd_vel_puber,())
    subscriber()

    

