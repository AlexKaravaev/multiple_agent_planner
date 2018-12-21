import rospy
import roslaunch
import os

if __name__=="__main__":
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    _dir = os.path.dirname(__file__)

    # kill old nodes and launch new roslaunch file
    os.system("rosnode kill /robosim")
    os.system("rosnode kill /map_server")
    os.system("rosnode kill /rviz")
    os.system("rosnode kill /stageros")

    os.system("gnome-terminal -- roslaunch multi_robot_sim robot_diff_drive_in_stage.launch")
