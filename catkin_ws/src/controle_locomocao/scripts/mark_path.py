#! /usr/bin/env python3
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point


position_ = Point()

def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    rospy.loginfo(position_)
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    
    path.header = msg.header
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose = msg.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)



def main():
    global path
    global path_pub
    
    path = Path()
    rospy.init_node('mark_path')

    rospy.sleep(1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    
    rospy.spin()


if __name__ == '__main__':
    main()