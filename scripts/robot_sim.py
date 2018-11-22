#!/usr/bin/env python3
import math
import threading
import rospy
import sys
import fileinput
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
import re
import random
import os

class RoboSim:
    def __init__(self):
        self.ask_n()
        self.spawn_points = []
        self.spawn_poses  = []

        rospy.init_node("robosim")
        rospy.Subscriber("initial_pose", PoseWithCovarianceStamped,self.spawn_topic_callback)
        self.array_pub = rospy.Publisher('/poses', PoseArray, queue_size=1)
        
        rospy.spin()
        
        # it's kostyl, but i will do it later more efficient
        self.open_spawn_points = [(x,y) for x in range(1,5) for y in range(1,5)]
        for x in range(1,5):
            self.open_spawn_points.append((x,9))
        
        self.dir = os.path.dirname(__file__)

    def ask_n(self):
        self.n_of_robots = int(input("N of robots: "))

    # Generate launch file given number of robots
    def generate_launch_file(self):
        begin = open('./src/begin_launch.txt', 'r')
        robot = open('./src/robot.txt', 'r+')
        end = open('./src/end_launch.txt', 'r+')
        final_launch = open('../launch/robot_diff_drive_in_stage.launch', 'w')
    
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
        begin = open('./src/begin_stage.txt', 'r')
        final_stage = open('../stage/maze_diff_drive.world', 'w')
        
        final_stage.write(begin.read())
        
        
        for i in range(self.n_of_robots):
            random_point = random.choice(self.open_spawn_points)
            pop_indx = self.open_spawn_points.index(random_point)
            self.open_spawn_points.pop(pop_indx)
            stage_robot = 'pr2(pose [' + str(random_point[0]) +' ' + str(random_point[1]) + ' 0 0] name "pr2_' + str(i) +'" color "red" )\n'
            final_stage.write(stage_robot)
        
        begin.close()
        final_stage.close()

    def generate_rviz_file(self):
        begin = open('./src/begin_rviz.txt', 'r')
        robot = open('./src/robot_rviz.txt', 'r+')
        end = open('./src/end_rviz.txt', 'r+')
    
        final_rviz = open('../cfg/multi_robot.rviz', 'w')
    
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

    def spawn_topic_callback(self, data):
        self.spawn_points.append((data.position.x,data.position.y,data.orientation.z))
        self.spawn_poses.append(data)
        
        msg = PoseArray()
        msg.header.frame_id = "robots"
        msg.header.stamp = rospy.Time.now()

        for pose in self.spawn_poses:
            msg.poses.append(Pose(pose.position, pose.orentation))
    
        self.array_pub.publish(msg)

if __name__=="__main__":
    sim = RoboSim()
    sim.generate_launch_file()
    sim.generate_stage_file()
    sim.generate_rviz_file()
