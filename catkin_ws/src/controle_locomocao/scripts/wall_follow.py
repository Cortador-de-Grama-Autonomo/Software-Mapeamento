#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import matplotlib.pyplot as plt
from s_walk import main as swalk

import math

matriz = [[0 for x in range(40)] for y in range(40)] 
active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
position_ = 0
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_odom(msg):
    global position_
    global yaw_
    
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

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 4),
        'fright': min(min(msg.ranges[144:287]), 4),
        'front':  min(min(msg.ranges[288:431]), 4),
        'fleft':  min(min(msg.ranges[432:575]), 4),
        'left':   min(min(msg.ranges[576:719]), 4),
    }
    
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        state_ = state

def take_action():
    global regions_, state_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 2.5
    if state_ != 3:
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            change_state(0)
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
    msg.linear.x = 0.6
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.6
    return msg

def follow_the_wall():
    global regions_, matriz, position_

    if matriz[int(position_.x) + 15][int(position_.y) + 15] != 2:
        matriz[int(position_.x) + 15][int(position_.y) + 15] = 2
    
    msg = Twist()
    msg.linear.x = 0.6
    return msg

def check_formed_map():
    global state_, matriz
    first_col = 50
    last_col = 0

    first_line = 50
    last_line = 0

    m = matriz.copy()
    for x, line in enumerate(m):
        column = [row[x] for row in m]
        if column.count(2) > 3:
            first_col = min(first_col, x)
            last_col = max(last_col, x)
        
        if line.count(2) > 3:
            first_line = min(first_line, x)
            last_line = max(last_line, x)

    column_list = [x for x in range(first_col, last_col)]
    line_list = [x for x in range(first_line, last_line)]
    
    # print("MATRIZ")
    # m_print = matriz.copy()
    # for line in m_print:
    #     for n, el in enumerate(line):
    #         if el == 0:
    #             line[n] = '-'
    #     print ('  '.join(map(str, line)))

    if 49 > (last_line - first_line ) > 5 and 49 > (last_col - first_col) > 5:
        flag = True
        for x, line in enumerate(m):
            column = [row[x] for row in m]
            
            # COLUMN MAPS 4
            if x in column_list:
                if column.count(2) < 2:
                    flag = False
                #else:
                    # for l in range(first_line, last_line):
                    #     for c in range(first_col, last_col):
                    #         if column[l] == 3:
                    #             rospy.loginfo("Coluna Preenchida para 5")
                    #             matriz[l][c] = 5
                    #         elif column[l] == 0:
                    #             rospy.loginfo("Coluna Preenchida para 4")
                    #             matriz[l][c] = 4


            # LINE MAPS 3
            if x in line_list:
                if line.count(2) < 2:
                    flag = False
                # else:
                #     for l in range(first_line, last_line):
                #         for c in range(first_col, last_col):
                #             if line[c] == 4:
                #                 rospy.loginfo("Linha Preenchida para 5")
                #                 matriz[l][c] = 5
                #             elif line[c] == 0:
                #                 rospy.loginfo("Linha Preenchida para 3")
                #                 matriz[l][c] = 3

        rospy.loginfo("Flag")
        rospy.loginfo(flag)
        if flag:
            state_ = 3
            for l in range(first_line, last_line):
                for c in range(first_col, last_col):
                    if matriz[l][c] != 2:
                        matriz[l][c] = 5
            
            m = matriz.copy()
            print("Matriz")
            for line in m:
                for n, el in enumerate(line):
                    if el == 0:
                        line[n] = '-'
                print ('  '.join(map(str, line)))
        


def main():
    global pub_, active_, matriz, position_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/cortador/laser/scan', LaserScan, clbk_laser)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    position_ = 0
    state_ == 3
    #srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            check_formed_map()
        elif state_ == 3:
            plt.imshow(matriz)
            plt.show()
            swalk(matriz)
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()