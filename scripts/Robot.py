#!/usr/bin/env python
from math import sqrt, pi, atan2
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, Quaternion, Point
from nav_msgs.msg import Odometry, Path
import tf
from tf.transformations import euler_from_quaternion


class Robot:

    def __init__(self):
        """"
        sets up publishers and subscribers, and creates global variables
        to store the robot pose and navigation target
        """
        rospy.init_node('Robot', anonymous=True)

        self.velPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
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
        self.target = Point()
        self.target.x = rospy.get_param('~x_pos', 0.0)
        self.target.y = rospy.get_param('~y_pos', 0.0)
        self.count = 0
        print("Robot initialized")


    # receive and set the target point that the robot should travel to
    # then, start navigating towards it
    def set_target(self, target_pose):
        self.target = target_pose.position  # drop orientation, we don't use it
        print("\n" + "navigation target updated:\n" + str(self.target))
        self.nav_to_point()


    def nav_to_point(self):
        goalx = self.target.x
        goaly = self.target.y
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


    def drive_straight(self, distance_to_travel):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance_to_travel: float
        :param speed: speed to drive
        :param distance_to_travel: distance to be traveled
        :return:
        """
        # set starting and target positions, and the distance driven so far
        start_pos = sqrt((self.px * self.px) + (self.py * self.py))
        curr_pos = start_pos
        distance_driven = abs(curr_pos - start_pos)

        # make a twist message to command the robot at the launch file's speed
        vel_msg = Twist()
        vel_msg.linear.x = rospy.get_param('~drive_speed', 0.1)

        # print debugging information
        print("\n" + "entered drive_straight")
        print("starting location is:\n" + str(start_pos))
        print("target location is:\n" + str(curr_pos))

        # get the tolerance from the launch file, drive until within tolerance
        tolerance = rospy.get_param('~drive_threshold', 0.05)
        while tolerance < abs(distance_to_travel - distance_driven):

            #publish driving command
            self.velPublisher.publish(vel_msg)

            # calculate the distance the robot has driven so far
            currpos = sqrt((self.px * self.px) + (self.py * self.py))
            distance_driven = abs(curr_pos - start_pos)

            # end the code nicely if shut down
            if rospy.is_shutdown():
                break

        # stop driving
        vel_msg = Twist()
        self.velPublisher.publish(vel_msg)
        print("\n" + "finished driving")
        print("Desired distance: " + str(distance))
        print("Actual: " + str(currdist))
        print("Error: " + str(abs(distance_to_travel - distance_driven)) + "\n\n")


    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        # set initial position
        currangle = self.yaw % (2 * pi)  # get rid of gross pi/ -pi thing
        endangle = (currangle + angle) % (2 * pi)

        threshold = rospy.get_param('~rotate_threshold', 0.03)

        # make a twist message to command the robot to turn
        vel_msg = Twist()
        vel_msg.angular.z = rospy.get_param('~rotate_speed', 0.15)

        # print debugging information
        print("\n" + "entered rotate")
        print("starting angle is: " + str(currangle))
        print("target angle is: " + str(endangle))

        # move in that direction until you've rotated the total specified angle
        while threshold < abs(endangle - currangle):
            self.velPublisher.publish(vel_msg)
            currangle = self.yaw % (2 * pi)  # get rid of gross pi/ -pi thing

            # end the code nicely if shut down
            if rospy.is_shutdown():
                break

        # stop rotating
        vel_msg.angular.z = 0
        self.velPublisher.publish(vel_msg)
        print("\n" + "finished rotating")
        print("Desired angle: " + str(endangle))
        print("Actual: " + str(currangle))
        print("Error: " + str(abs(endangle - currangle)))


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
