
/* STD CPP DEPS*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <utility>

// Header for planner
#include "example_planner.h"

// We need to register this as BaseGlobalPlanner plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_planner::Example_Planner, nav_core::BaseGlobalPlanner)


namespace example_planner{

// Constructors
Example_Planner::Example_Planner(){}

Example_Planner::Example_Planner(const ros::NodeHandle &nh)
{
	NodeHandler = nh;
}

Example_Planner::Example_Planner(const std::string& name,costmap_2d::Costmap2DROS* costmap_ros){
	initialize(name, costmap_ros);
}

void Example_Planner::initialize(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros){
	if (!initizalized_)
	{
		costmap_ros_ = costmap_ros;
		costmap_ = costmap_ros_ -> getCostmap();
		
		ros::NodeHandle private_nh("~/" + name);

		std::pair <float, float> Origin;
		
		origin = std::make_pair(costmap_->getOriginX(), costmap_->getOriginY());

		// Width and height
		size = std::make_pair(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

		resolution = costmap_->getResolution();
		mapSize = size.first*size.second;
		tBreak = 1+1/(mapSize);
		value = 0;
		
		//Creating bool map
		OGM = new bool [mapSize];
		for (unsigned auto iy = 0; iy < size.second; iy++){
			for(unsigned auto ix = 0; ix < size.first; ix++){
				cost = static_cast<int>(costmap_->getCost(ix, iy));
				if (cost == 0):
					OGM[iy*size.first + ix] = true;
				else
					OGM[iy*size.first + ix] = false;
			}
		}
		
		ROS_INFO("Example planner initialized succesfully");
		initialized_ = true;
	}
	else
		ROS_WARN("It's already have been initialized");
}

bool Example_Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
	if (!initialized_){
		ROS_ERROR("First initialize planner, then call this method");
		return false;
	}
	ROS_DEBUG("Start: (%2f,%2f) and Goal: (%2f, %2f)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
	plan.clear();
	
	if (goal.header.frame_id != costmap_ros_ -> getGLobalFrameID()){
		ROS_ERROR("Planner is configured to accept goal in %s frame, but goal was sent in %s frame", costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
		return false;
	}
	
	tf::Stamped <tf::Pose> goal_tf;
	tf::Stamped <tf::Pose> start_tf;
	
	poseStampedMsgToTF(goal, goal_tf);
	poseStampedMsgToTF(start, start_tf);
	
	std::pair <float, float> coord_start = std::make_pair(start.pose.position.x, start.pose.position.y);
	
	std::pair <float, float> coord_goal = std::make_pair(goal.pose.positoon.x, goal.pose.position.y);
	
	getCoordinate(coord_start);
	getCoordinate(coord_goal);
	
	int startCell;
	int goalCell;
	
	if (isCellInsideMap(coord_start.first, coord_start.second) && isCellInsideMap(coord_goal.first, coord_goal.second)){
		start_cell = convertToCellIndex(coord_start);
		goalcell = convertToCellIndex(coord_goal);
	}
	else{
		ROS_WARN("Start or goal cell is out of the map");
		return false;
	}
	
	
