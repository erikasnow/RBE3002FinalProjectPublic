#!/usr/bin/env python
from math import sqrt, pi, atan2
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, Quaternion, Point
from nav_msgs.msg import Odometry, Path
import tf
from tf.transformations import euler_from_quaternion
from std_msgs import Bool


class Robot:

    def __init__(self):
        """"
        Set up the node here
        """
        rospy.init_node('Robot', anonymous=True)
        # we should be publishing the velocity and the goal pose, and receiving the current pose
        self.velPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odomSubscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        # self.rvizSubscriber = rospy.Subscriber('rviz_click', PoseStamped, self.nav_to_pose)
        # self.pathSubscriber = rospy.Subscriber('path', Path, self.handle_path)
        self.poseSubscriber = rospy.Subscriber('target', PoseStamped, self.set_target)
        self.poseSubscriberTest = rospy.Subscriber('test', Pose, self.set_target)

        # need to publish a message to let know that the planning node can send the next point
        # maybe this needs to be service instead? so that planning waits until it receives the message?
        self.donePublisher = rospy.Publisher('done', PoseStamped, queue_size=1)

        self.px = 0
        self.py = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # target point to travel to
        self.target = Point()
        self.count = 0
        print("made it through init")

    # set the global point that the robot should travel to
    def set_target(self, pose):
        print("")
        print("target updated")
        self.target = pose.position  # we don't really care about the orientation of the robot, so no Quaternion

    def nav_to_point(self):
        goalx = self.target.x
        goaly = self.target.y

        # rotate toward goal
        currx = self.px
        curry = self.py
        deltax = goalx - currx
        deltay = goaly - curry

        threshold = 0.15

        distance = sqrt((deltax * deltax) + (deltay * deltay))

        if threshold < (abs(distance)):
            print("change in dist: " + str(abs(distance)))
            angle = atan2(deltay, deltax)  # now have desired angle to rotate
            self.rotate(angle)

            # drive straight
            self.drive_straight(0.5, distance)

    def handle_move_to_target(self):
        if self.count < 2:
            self.nav_to_point()

        self.donePublisher.publish(True)  # this may need to be a service? Not sure...

    # deconstruct the path and call nav to pose for each one
    def handle_path(self, path):
        print("entered handle_path")
        print("")
        # delete first cell in the path so that Robot assumes it's at the start location
        # path.poses.pop(0)

        self.nav_to_pose(path.poses[0])
        path.poses.pop(0)

        for pose in path.poses:
            print("")
            print(pose.pose)
            self.astar_nav(pose)

        print("")
        print("end of handle_path")

    def astar_nav(self, goal):
        print("")
        print("entered astar_nav")

        goalx = goal.pose.position.x
        goaly = goal.pose.position.y

        # rotate toward goal
        currx = self.px
        curry = self.py
        deltax = goalx - currx
        deltay = goaly - curry
        angle = atan2(deltay, deltax)  # now have desired angle to rotate
        self.rotate(angle)

        # drive straight
        distance = sqrt((deltax * deltax) + (deltay * deltay))
        self.drive_straight(0.5, distance)

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a straight line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        print("")
        print("entered nav_to_pose")

        goalx = goal.pose.position.x
        goaly = goal.pose.position.y
        quat = goal.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        goalroll, goalpitch, goalyaw = euler_from_quaternion(q)

        # rotate toward goal
        currx = self.px
        curry = self.py
        deltax = goalx - currx
        deltay = goaly - curry
        angle = atan2(deltay, deltax)  # now have desired angle to rotate
        self.rotate(angle)

        # drive straight
        distance = sqrt((deltax * deltax) + (deltay * deltay))
        self.drive_straight(0.3, distance)

        # rotate to goal orientation
        finalangle = goalyaw - self.yaw
        self.rotate(finalangle)

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
        print("entered drive_straight")
        # set initial position
        startx = self.px
        starty = self.py
        startpos = sqrt((startx * startx) + (starty * starty))

        threshold = 0.05

        currpos = sqrt((self.px * self.px) + (self.py * self.py))
        change = currpos - startpos
        currdist = abs(change)
        vel_msg = Twist()
        while (threshold < (abs(distance - currdist))):  # see if this improves accuracy
            # start driving forward (i.e tell turtlebot to move at certain velocity -> publish a cmd_vel message)
            vel_msg.linear.x = speed
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.velPublisher.publish(vel_msg)

            # calculate distance between current location and initial location
            # update the values
            currpos = sqrt((self.px * self.px) + (self.py * self.py))
            change = currpos - startpos
            currdist = abs(change)

        # stop when set distance has been achieved (i.e publish a cmd_vel message w/ all zeroes)
        vel_msg.linear.x = 0
        print("Desired distance: " + str(distance))
        print("Actual: " + str(currdist))
        print("Error: " + str(abs(distance - currdist)))
        print("")
        self.velPublisher.publish(vel_msg)

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        print("entered rotate")
        # set initial position
        currangle = self.yaw % (2 * pi)  # get rid of gross pi/ -pi thing
        endangle = (currangle + angle) % (2 * pi)

        """
        # grab necessary direction
        if(endangle < currangle):
            direction = -1  # if angle was negative, move right
        else:
            direction = 1  # if angle was position, move left
        """

        threshold = 0.03  # allowed error in radians (1.7 degrees, currently)

        vel_msg = Twist()
        # move in that direction until you've rotated the total specified angle
        while threshold < abs(endangle-currangle):
            # start spinning robot in correct direction
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.3
            self.velPublisher.publish(vel_msg)

            # update the current angle
            currangle = self.yaw % (2 * pi)  # get rid of gross pi/ -pi thing

        # stop at desired angle
        vel_msg.angular.z = 0
        self.velPublisher.publish(vel_msg)
        print("Desired angle: " + str(endangle))
        print("Actual: " + str(currangle))
        print("Error: " + str(abs(endangle - currangle)))

    # grabs pose of the turtlebot
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
    while True:  # maybe add in if target hasn't changed in a certain number of iterations?
        goal = r.target
        r.handle_move_to_target()
        if goal == r.target:
            r.count += 1
        else:
            r.count = 0

    rospy.spin()
