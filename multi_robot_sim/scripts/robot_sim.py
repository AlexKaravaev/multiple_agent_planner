#!/usr/bin/env python3
import math
import rospy
import sys
import fileinput
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import roslaunch
import re
import random
import os
import time


class RoboSim:
    def __init__(self):

        self.spawn_points = []
        self.spawn_poses  = []
        self.asked = False

        rospy.init_node("robosim")
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped,self.spawn_topic_callback)
        self.array_pub = rospy.Publisher('/poses', PoseArray, queue_size=1)


        time.sleep(1)
        self.ask_n()

        self.dir = os.path.dirname(__file__)

        rospy.spin()

    #TEMP func. Going to rewrite it
    def quat_to_euler(self, quat):
        q0 = quat[0]
        q1 = quat[1]
        q2 = quat[2]
        q3 = quat[3]

        roll = math.atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2))
        pitch = math.asin(2*(q0*q2 - q3*q1))
        yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))

        return (roll, pitch, yaw)

    # Promts user for n of robots
    def ask_n(self):


        while True:
            try:
                self.n_of_robots = int(input("N of robots:"))
                break
            except ValueError:
                rospy.logwarn("Enter valid integer number of robots")

        self.temp_n = self.n_of_robots
        rospy.loginfo("Now set initial poses of robots in rviz")
        self.asked = True

    # Generate launch file given number of robots
    def generate_launch_file(self):
        begin = open(self.dir + '/src/begin_launch.txt', 'r')
        robot = open(self.dir + '/src/robot.txt', 'r+')
        end = open(self.dir + '/src/end_launch.txt', 'r+')
        final_launch = open(self.dir[:-len('scripts')] + 'launch/robot_diff_drive_in_stage.launch', 'w')

        final_launch.write(begin.read())
        new = robot.read()
        for i in range(self.n_of_robots):
            final_launch.write(new.replace('robot_0','robot_'+str(i)))

        final_launch.write(end.read())
        begin.close()
        robot.close()
        end.close()
        final_launch.close()

    # Generate stage file given n of robots and their poses
    def generate_stage_file(self):
        begin = open(self.dir + '/src/begin_stage.txt', 'r')
        final_stage = open(self.dir[:-len('scripts')] + 'stage/maze_diff_drive.world', 'w')

        final_stage.write(begin.read())


        for i in range(self.n_of_robots):
            point = self.spawn_points[i]
            stage_robot = 'pr2(pose [' + str(point[0]) +' ' + str(point[1]) +' 0' + ' ' + str(math.degrees(point[2][0])) + '] name "pr2_' + str(i) +'" color "red" )\n'
            final_stage.write(stage_robot)

        begin.close()
        final_stage.close()

    # Generate rviz file based on n of robots
    def generate_rviz_file(self):
        begin = open(self.dir + '/src/begin_rviz.txt', 'r')
        robot = open(self.dir + '/src/robot_rviz.txt', 'r+')
        end = open(self.dir + '/src/end_rviz.txt', 'r+')

        final_rviz = open(self.dir[:-len('scripts')] + 'cfg/multi_robot.rviz', 'w')

        final_rviz.write(begin.read())
        new = robot.read()
        for i in range(self.n_of_robots):
            temp = new.replace('robot_0', 'robot_'+str(i))

            final_rviz.write(temp.replace('Robot 0', 'Robot ' + str(i)))

        final_rviz.write(end.read())
        begin.close()
        robot.close()
        end.close()
        final_rviz.close()

    # Generate all sys files
    def generate_files(self):
        self.generate_launch_file()
        self.generate_stage_file()
        self.generate_rviz_file()
        self.launch_general_node()
        sys.exit()

    def launch_general_node(self):
        os.system("python3 " + self.dir + "/sim_start.py")

    # Callback method for listening to robots poses topic
    def spawn_topic_callback(self, data):
        if self.asked:
            if self.temp_n != 0:
                self.spawn_points.append((data.pose.pose.position.x,data.pose.pose.position.y,self.quat_to_euler((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))))

                self.temp_n -= 1
                self.spawn_poses.append(data)
                rospy.loginfo("Got data from pose topic (x,y,(roll,pitch,yaw))\n{}".format(self.spawn_points[-1]))
                msg = PoseArray()
                msg.header.frame_id = "/map"
                msg.header.stamp = rospy.Time.now()

                for _pose in self.spawn_poses:
                    msg.poses.append(Pose(_pose.pose.pose.position, _pose.pose.pose.orientation))
                self.array_pub.publish(msg)

            if self.temp_n == 0:
                self.generate_files()

        else:
            rospy.logwarn("Enter number of robots first!")
            print("N of robots: ")

if __name__=="__main__":
    sim = RoboSim()
