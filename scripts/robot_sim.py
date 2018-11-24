#!/usr/bin/env python3
import math
#import threading
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


class RoboSim:
    def __init__(self):
        self.ask_n()
        self.spawn_points = []
        self.spawn_poses  = []

        rospy.init_node("robosim")
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped,self.spawn_topic_callback)
        self.array_pub = rospy.Publisher('/poses', PoseArray, queue_size=1)
        
        
        
        # it's kostyl, but i will do it later more efficient
        self.open_spawn_points = [(x,y) for x in range(1,5) for y in range(1,5)]
        for x in range(1,5):
            self.open_spawn_points.append((x,9))
        
        self.dir = os.path.dirname(__file__)
        
        rospy.spin()

    def ask_n(self):
        self.n_of_robots = int(input("N of robots: "))
        self.temp_n = self.n_of_robots

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
    
    def generate_stage_file(self):
        begin = open(self.dir + '/src/begin_stage.txt', 'r')
        final_stage = open(self.dir[:-len('scripts')] + 'stage/maze_diff_drive.world', 'w')
        
        final_stage.write(begin.read())
        
        
        for i in range(self.n_of_robots):
            random_point = random.choice(self.open_spawn_points)
            pop_indx = self.open_spawn_points.index(random_point)
            self.open_spawn_points.pop(pop_indx)
            point = self.spawn_points[i]
            stage_robot = 'pr2(pose [' + str(point[0]) +' ' + str(point[1]) +' '+str(point[2]) + ' 0] name "pr2_' + str(i) +'" color "red" )\n'
            final_stage.write(stage_robot)
        
        begin.close()
        final_stage.close()

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
    
    def generate_files(self):
        self.generate_launch_file()
        self.generate_stage_file()
        self.generate_rviz_file()
        self.launch_general_node()
        sys.exit()
    
    def launch_general_node(self):
        os.system("python3 " + self.dir + "/sim_start.py")
    
    def spawn_topic_callback(self, data):
        if self.temp_n != 0:
            self.spawn_points.append((data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z))
            self.temp_n -= 1
            self.spawn_poses.append(data)
            rospy.loginfo("Got data from pose topic\n{}".format(data)) 
            msg = PoseArray()
            msg.header.frame_id = "/map"
            msg.header.stamp = rospy.Time.now()

            for _pose in self.spawn_poses:
                msg.poses.append(Pose(_pose.pose.pose.position, _pose.pose.pose.orientation))
            self.array_pub.publish(msg)

        if self.temp_n == 0:
            self.generate_files()
            

if __name__=="__main__":
    sim = RoboSim()
