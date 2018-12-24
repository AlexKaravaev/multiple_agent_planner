#!/usr/bin/env python
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
from tf.transformations import euler_from_quaternion

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

    # Promts user for n of robots
    def ask_n(self):


        while True:
            try:
                self.n_of_robots = int(input("N of robots:"))
                if self.n_of_robots <= 0:
                    rospy.logwarn("N of robots should be > 0")
                    continue
                break
            except ValueError:
                rospy.logwarn("Enter valid integer number of robots")

        self.temp_n = self.n_of_robots
        rospy.loginfo("Now set initial poses of robots in rviz")
        rospy.set_param("n_of_robots", self.n_of_robots)
        self.asked = True

    # Generate launch file given number of robots
    def generate_launch_file(self):
        begin = open(self.dir + '/src/begin_launch.txt', 'r')
        robot = open(self.dir + '/src/robot_one_launch.txt', 'r+') if self.n_of_robots==1 else open(self.dir + '/src/robot.txt', 'r+')
        end = open(self.dir + '/src/end_launch.txt', 'r+')
        final_launch = open(self.dir[:-len('scripts')] + 'launch/robot_diff_drive_in_stage.launch', 'w')

        final_launch.write(begin.read())
        new = robot.read()

        if self.n_of_robots == 1:
            final_launch.write(new.replace('robot_0',''))
        else:
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

        if self.n_of_robots == 1:
            point = self.spawn_points[0]
            stage_robot = 'pr2(pose [' + str(point[0]) +' ' + str(point[1]) +' 0' + ' ' + str(math.degrees(point[2][0])) + '] name "pr2' +'" color "red" )\n'
            final_stage.write(stage_robot)
        else:
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

        if self.n_of_robots == 1:
            temp = new.replace('robot_0', '')
            final_rviz.write(begin.read().replace('robot_0', ''))
            final_rviz.write(temp.replace('Robot 0', ''))
            final_rviz.write(end.read().replace('robot_0',''))
        else:
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
                orientation_list = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
                self.spawn_points.append((data.pose.pose.position.x,data.pose.pose.position.y,
                                            euler_from_quaternion(orientation_list)))

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
            print "N of robots: "

if __name__=="__main__":
    sim = RoboSim()
