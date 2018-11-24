import rospy
import roslaunch
import os

if __name__=="__main__":
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    _dir = os.path.dirname(__file__)
    
    os.system("rosnode kill /robosim")
    os.system("rosnode kill /map_server")
    os.system("rosnode kill /rviz")
    os.system("rosnode kill /stageros")

    os.system("gnome-terminal -- roslaunch detached_planner robot_diff_drive_in_stage.launch")
    #launch = roslaunch.parent.ROSLaunchParent(uuid,
    #            [(_dir[:-len('scripts')] + 'launch/robot_diff_drive_in_stage.launch')])

    #launch.start()
    #rospy.loginfo("Starting real similator'")
    #rospy.spin()
