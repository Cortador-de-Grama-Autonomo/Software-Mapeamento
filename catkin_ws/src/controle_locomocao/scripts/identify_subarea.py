#! /usr/bin/env python3

# import ros stuff
from numpy.core.numeric import count_nonzero
import rospy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


position_ = Point()
state = 0

w, h = 50, 50
matriz = [[0 for x in range(w)] for y in range(h)] 


def clbk_odom(msg):
    # position
    position_ = msg.pose.pose.position
    register_border()

def clbk_range_r(msg):
    range_r = min(msg.range, 1)
    if range_r < 1:
        register_border()

def clbk_range_l(msg):
    range_l = min(msg.range, 1)
    if range_l < 1:
        register_border()

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
        #rospy.loginfo(front)
    
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def register_border():
    rospy.loginfo(matriz[49][49])
    rospy.loginfo(matriz[int(position_.x) + 20][int(position_.y) + 20])
    if matriz[int(position_.x) + 20][int(position_.y) + 20] != 1:
        matriz[int(position_.x) + 20][int(position_.y) + 20] = 1
        state = 1

def main():
    global pub
    global contador
    global position_
    global matriz
    global state

    rospy.init_node('reading_laser')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    sub_right = rospy.Subscriber('/cortador/sonar/scan_right', Range, clbk_range_r)
    sub_left = rospy.Subscriber('/cortador/sonar/scan_left', Range, clbk_range_l)
    sub = rospy.Subscriber('/cortador/laser/scan', LaserScan, clbk_laser)

    if state > 15: 
        for i in range(0,49):
            for j in range(0,49):
                rospy.loginfo(matriz[i][j])

    rospy.spin()

if __name__ == '__main__':
    main()
