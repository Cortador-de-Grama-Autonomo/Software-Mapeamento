#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from nav_msgs.msg import Odometry
from tf import transformations


class RobotControl():

    def stop_robot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
    
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()

    def get_inputs_rotate(self, clockwise_yn):
        if clockwise_yn == "y":
            self.clockwise = True
        if clockwise_yn == "n":
            self.clockwise = False

        return [5, 90]

    def convert_degree_to_rad(self, speed_deg, angle_deg):

        self.angular_speed_r = speed_deg * 3.1415 / 180
        self.angle_r = angle_deg * 3.1415 / 180
        return [self.angular_speed_r, self.angle_r]

    def rotate(self, clockwise_yn, msg, vel, rate, odom):
        self.cmd = msg
        self.vel_publisher = vel
        self.rate = rate

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        # Convert speed and angle to radians
        speed_d, angle_d = self.get_inputs_rotate(clockwise_yn)
        self.convert_degree_to_rad(speed_d, angle_d)

        # Check the direction of the rotation
        if self.clockwise:
            self.cmd.angular.z = -abs(self.angular_speed_r)
        else:
            self.cmd.angular.z = abs(self.angular_speed_r)

        # t0 is the current time
        t0 = rospy.Time.now().secs

        current_angle = 0
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        yaw_ = euler[2]

        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (-yaw_ < self.angle_r):
            quaternion = (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w)
            euler = transformations.euler_from_quaternion(quaternion)
            yaw_ = euler[2]

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            # t1 is the current time
            t1 = rospy.Time.now().secs
            # Calculate current angle
            rospy.loginfo(yaw_)
            rospy.loginfo(self.angle_r)
            current_angle = self.angular_speed_r * (t1 - t0)
            self.rate.sleep()

        # set velocity to zero to stop the robot
        rospy.loginfo("Parou")
        return self.cmd


# if __name__ == '__main__':
#     robotcontrol_object = RobotControl()
#     try:
#         res = robotcontrol_object.rotate()
#     except rospy.ROSInterruptException:
#         pass