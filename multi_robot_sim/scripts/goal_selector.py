#!/usr/bin/env python
import rospy
import roslaunch
import os
import sys
from geometry_msgs.msg import PoseStamped

class GoalSelector:

    def __init__(self):

        self.goal_poses = []

        rospy.init_node("goal_selector")
        rospy.Subscriber("goals", PoseStamped, self.goal_topic_callback,queue_size=2)

        self.goal_publishers = []
        self.goals = []


        self.n_of_robots = rospy.get_param("n_of_robots")

        # Create publisher for each of robot
        # FIXME maybe there is better way to do it
        for i in range(self.n_of_robots):
            move_base_str = "/move_base_simple/goal"
            self.goal_publishers.append(rospy.Publisher("/robot_"+str(i)+move_base_str, PoseStamped, queue_size=2))

        rospy.spin()


    # Listens for goal topic and append it to goal array
    def goal_topic_callback(self, data):
        rospy.loginfo("Got data from goal topic (x,y,(roll,pitch,yaw))\n{}".format(data))
        self.goals.append(data)

        # if goals are set it's time to send it to move_base_node of each robot
        if len(self.goals) == self.n_of_robots:
            self.publish_goals()

    # Publish goals to each of move_base_node
    def publish_goals(self):
        for i, pub in enumerate(self.goal_publishers):
            pub.publish(self.goals[i])
        self.goals = 0
        
if __name__ == '__main__':
    gs = GoalSelector()
