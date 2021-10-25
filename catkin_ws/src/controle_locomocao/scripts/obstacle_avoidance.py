#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None


def clbk_laser(msg):
    front = min(min(msg.ranges[0:20]), 2)
    take_action(front)


def take_action(front):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if front >= 2:
        state_description = 'caso 1 - sem obstaculos'
        linear_x = 0.6
        angular_z = 0
    elif front < 2:
        state_description = 'case 2 - obstáculo a frente'
        linear_x = 0
        angular_z = 1.0
    else:
        state_description = 'não identificado'
        rospy.loginfo(front)
    
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
        


def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/cortador/laser/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
