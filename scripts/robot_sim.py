import math
import threading
import rospy
import sys
import fileinput
from geometry_msgs.msg import PointStamped
import re

class RoboSim:
    def __init__(self):
        self.ask_n()

    def ask_n(self):
        self.n_of_robots = int(input("N of robots: "))

    # Generate launch file given number of robots
    def generate_launch_file(self):
        begin = open('./src/begin_launch.txt', 'r')
        robot = open('./src/robot.txt', 'r+')
        end = open('./src/end_launch.txt', 'r+')
        final_launch = open('final_launch.launch', 'w')
    
        final_launch.write(begin.read()) 
        new = robot.read() 
        for i in range(self.n_of_robots):
            final_launch.write(new.replace('robot_0','robot_'+str(i)))
        
        final_launch.write(end.read())
        begin.close()
        robot.close()
        end.close()
        final_launch.close()


if __name__=="__main__":
    rospy.init_node('robot_setter')
    sim = RoboSim()
    sim.generate_launch_file()


