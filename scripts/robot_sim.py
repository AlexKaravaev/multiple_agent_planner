import math
import threading
import rospy
import sys
import fileinput
from geometry_msgs.msg import PointStamped
import re
import random

class RoboSim:
    def __init__(self):
        self.ask_n()
        
        # it's kostyl, but i will do it later more efficient
        self.open_spawn_points = [(x,y) for x in range(1,5) for y in range(1,5)]
        for x in range(1,5):
            self.open_spawn_points.append((x,9))
        
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

if __name__=="__main__":
    rospy.init_node('robot_setter')
    sim = RoboSim()
    sim.generate_launch_file()
    sim.generate_stage_file()
    sim.generate_rviz_file()
