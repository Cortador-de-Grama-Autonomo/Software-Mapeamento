#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ConverVelocities():
    def __init__(self):
        self.sub = rospy.Subscriber('/twist_vels', Twist, self.callback)
        self.right_wheel_pub = rospy.Publisher('/right_wheel_controller/command', Float64)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_controller/command', Float64)
        self.twist_vels = Twist()
        self.vr = Float64()
        self.vl = Float64()
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        self.twist_vels = msg
    
    def pub_wheel_vels(self):
        while not rospy.is_shutdown():
            self.convert_velocities()
            self.right_wheel_pub.publish(self.vr)
            self.left_wheel_pub.publish(self.vl)
            self.rate.sleep()

    def convert_velocities(self):
        # L = 0.3
        # R = 0.1
        self.vr = ((2*self.twist_vels.linear.x) + (self.twist_vels.angular.z*0.3))/(2*0.1)
        self.vl = ((2*self.twist_vels.linear.x) - (self.twist_vels.angular.z*0.3))/(2*0.1)

if __name__ == '__main__':
    rospy.init_node('convert_vels_node', anonymous=True)
    cv = ConverVelocities()
    cv.pub_wheel_vels()  
    
    
    
    #------------------------------------------
    
    
    
import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from simple_robot_gazebo.msg import encoders
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:

    def __init__(self):
        self.ticks_sub = rospy.Subscriber('/encoders', encoders, self.getTicks)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_broadcaster = TransformBroadcaster()
        self.odom = Odometry()
        self.rate = rospy.Rate(1)
        self.lastLeftTicks = 0
        self.lastRightTicks = 0
        self.currentLeftTicks = 0
        self.currentRightTicks = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.L = 0.3
        self.R = 0.1
        self.N = 360
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        rospy.wait_for_message('/clock', Clock)
        self.updatePose()

    def getTicks(self, msg):
        self.currentLeftTicks = msg.encoderTicks[0]
        self.currentRightTicks = msg.encoderTicks[1]
        
    def updatePose(self):

        while not rospy.is_shutdown():

            delta_l = self.currentLeftTicks - self.lastLeftTicks
            delta_r = self.currentRightTicks - self.lastRightTicks
            d_l = 2 * pi * self.R * delta_l / self.N
            d_r = 2 * pi * self.R * delta_r / self.N

            self.lastLeftTicks = self.currentLeftTicks
            self.lastRightTicks = self.currentRightTicks
            
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()

            v = ((d_r + d_l) / 2) / dt
            w = ((d_r - d_l) / self.L) / dt

            # compute odometry in a typical way given the velocities of the robot
            delta_x = v * cos(self.th)
            delta_y = v * sin(self.th)
            delta_th = w

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "link_chassis",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "link_chassis"
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            # publish the message
            self.odom_pub.publish(odom)

            self.last_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pub_odom')
    oc = OdometryClass()
    rospy.spin()
