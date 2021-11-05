#! /usr/bin/env python3

# import ros stuff
import rospy
from rospy.core import loginfo
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

matriz = [[0 for x in range(40)] for y in range(40)] 
ativo_flag  = 0
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def clbk_odom(msg):
    global position_
    global yaw_
    global matriz
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    
    if (matriz[int(position_.x) + 15][int(position_.y) + 15] != 1 and
        matriz[int(position_.x) + 15][int(position_.y) + 15] != 2):
        matriz[int(position_.x) + 15][int(position_.y) + 15] = 1
        
        m = matriz.copy()
        print("Matriz")
        for line in m:
            for n, el in enumerate(line):
                if el == 0:
                    line[n] = '-'
            print ('  '.join(map(str, line)))


def clbk_laser(msg):
    global regions_
    global ativo_flag
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 4),
        'fright': min(min(msg.ranges[144:287]), 4),
        'front':  min(min(msg.ranges[288:431]), 4),
        'fleft':  min(min(msg.ranges[432:575]), 4),
        'left':   min(min(msg.ranges[576:719]), 4),
    }
    
    take_action(ativo_flag)

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        state_ = state

def take_action(ativo):
    global regions_
    global ativo_flag
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    #ativo = ativo_flag
    
    state_description = ''
    
    d = 1.25
    rospy.loginfo("Ativo")
    rospy.loginfo(ativo_flag)
    if ativo == 0:
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] < 2.00:
            state_description = 'case 3 - fright'
            change_state(2)
        elif regions['front'] > d and regions['fleft'] < 2.00 and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            change_state(2)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            change_state(1)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.8
    msg.angular.z = 0
    return msg

def turn_left():
    global turn, matriz
    
    msg = Twist()
    msg.linear.x = 0.08
    duration = 10
    msg.angular.z = turn * (math.pi*2/2/duration)

    pub_.publish(msg)
    rospy.sleep(duration)

    #msg.linear.x = 0.08
    #duration = 5
    #msg.angular.z = turn * (math.pi*2/4/duration)

    #pub_.publish(msg)
    #rospy.sleep(duration)

    if turn == 1:
        turn = -1
    else:
        turn = 1
    
    print("Setando velocidade como zero")
    msg.linear.x = 0
    msg.angular.z = 0
    pub_.publish(msg)

    return 0


def follow_the_wall():
    global matriz
    
    msg = Twist()
    msg.linear.x = 0.6
    msg.angular.z = 0.06
    return msg

def main():
    global pub_, state_, turn, matriz
    #matriz = m
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    sub = rospy.Subscriber('/cortador/laser/scan', LaserScan, clbk_laser)

    turn = 1
    
    #srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        global ativo_flag
        ativo_flag = 0
        #rospy.loginfo(state_)
        msg = Twist()
        ativo_flag = 0
        if state_ == 0:
            msg = find_wall()
            pub_.publish(msg)
        elif state_ == 1:
            ativo_flag = 1
            ativo_flag = turn_left()
            state_ = 3
            ativo_flag = 0
        elif state_ == 2:
            msg = follow_the_wall()
            pub_.publish(msg)
        else:
            rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
    main()