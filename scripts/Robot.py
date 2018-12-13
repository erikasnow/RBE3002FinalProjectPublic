#!/usr/bin/env python
from math import sqrt, atan2
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, Quaternion, Point
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class Robot:

    def __init__(self):
        """"
        sets up publishers and subscribers, and creates global variables
        to store the robot pose and navigation target
        """
        rospy.init_node('Robot', anonymous=True)

        self.velPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # publish pose when reached
        self.donePublisher = rospy.Publisher('done', Pose, queue_size=1)

        self.odomSubscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # subscribe to navigation targets from main
        self.poseSubscriber = rospy.Subscriber('target', Pose, self.set_target)

        # variables to store the robot's pose, initialized from the launch file
        self.px = rospy.get_param('~x_pos', 0)
        self.py = rospy.get_param('~x_pos', 0)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # target point to travel to, initialized from the launch file
        self.target = Pose()
        self.target.position.x = rospy.get_param('~x_pos', 0.0)
        self.target.position.y = rospy.get_param('~y_pos', 0.0)
        self.count = 0
        print("Robot initialized")

    # receive and set the target point that the robot should travel to
    # then, start navigating towards it
    def set_target(self, target_pose):
        self.target = target_pose
        print("\n" + "navigation target updated:\n" + str(self.target))
        self.nav_to_point()

    def nav_to_point(self):
        goal = self.target
        goalx = goal.position.x
        goaly = goal.position.y
        currx = self.px
        curry = self.py

        deltax = goalx - currx
        deltay = goaly - curry

        distance = sqrt((deltax * deltax) + (deltay * deltay))
        print "in nav_to_point, distance to target is: " + str(distance)

        threshold = rospy.get_param('~dist_threshold', 0.15)

        if threshold < abs(distance):
            angle = atan2(deltay, deltax)
            self.rotate(angle)
            self.drive_straight(distance)

        # let main node know robot has finished traveling
        self.donePublisher.publish(goal)
        print("robot published done")

    def drive_straight(self, distance_to_travel):
        """
        Make the turtlebot drive straight for a given distance
        :type distance_to_travel: float
        :param distance_to_travel: distance to be traveled
        :return:
        """
        # set starting and current positions, and the distance driven so far
        start_x = self.px
        start_y = self.py
        distance_driven = 0.0

        # print debugging information
        print("\n" + "entered drive_straight")
        print("starting location: (" + str(start_x) + " , " + str(start_y) + ")")
        print("distance to travel: " + str(distance_to_travel))

        # make a twist message to command the robot at the launch file's speed
        vel_msg = Twist()
        vel_msg.linear.x = rospy.get_param('~drive_speed', 0.1)

        # get the tolerance from the launch file, drive until within tolerance
        tolerance = rospy.get_param('~drive_tolerance', 0.05)
        while tolerance < abs(distance_to_travel - distance_driven) and not rospy.is_shutdown():
            self.velPublisher.publish(vel_msg)
            distance_driven = sqrt((start_y - self.py)**2 + (start_x - self.px)**2)

        # stop driving
        vel_msg = Twist()
        self.velPublisher.publish(vel_msg)
        print("\n" + "finished driving")
        print("Desired distance: " + str(distance_to_travel))
        print("Actual: " + str(distance_driven))
        print("Error: " + str(abs(distance_to_travel - distance_driven)) + "\n\n")

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        # print debugging information
        print("\n" + "entered rotate")
        print "target angle is: " + str(angle)
        print "current yaw is:\t " + str(self.yaw)

        # make a twist message to command the robot to turn
        vel_msg = Twist()
        vel_msg.angular.z = rospy.get_param('~rotate_speed', 0.15)
        if angle < 0: # if it would be easier to turn the other way, change sign
            vel_msg.angular.z = -vel_msg.angular.z

        # rotate in that direction to within tolerance of the specified angle
        tolerance = rospy.get_param('~rotate_tolerance', 0.03)
        while abs(self.yaw - angle) > tolerance and not rospy.is_shutdown():
            self.velPublisher.publish(vel_msg)

        # stop rotating
        vel_msg.angular.z = 0
        self.velPublisher.publish(vel_msg)
        print("\n" + "finished rotating")
        print("Desired angle: " + str(angle))
        print("Actual: " + str(self.yaw))
        print("Error: " + str(abs(angle - self.yaw)))

    # receive updates for the pose of the turtlebot
    def odom_callback(self, msg):
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)


if __name__ == '__main__':
    print("make robot")
    r = Robot()
    rospy.sleep(1)  # make sure the robot has time to receive init values

    rospy.spin()
