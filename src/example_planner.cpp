
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

		std::pair <float, float> origin;
		
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
	
	if (isStartAndGoalCellsValid(startCell, goalCell)){
		std::vector<int> bestPath;
		bestPath.clear();
		bestPath = AStarPlanner(startCell,goalCell);
		
		//If path exist
		if (bestPath.size() > 0){
			
			for (auto i = 0; i < bestPath.size(); i++){
				float x = 0.0;
				float y = 0.0;
			
				int index = bestPath[i];
				
				convertToCoordinate(index, x, y);	
				
				geometry_msgs::PoseStamped pose = goal;
				
				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;
				

				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;

				plan.push_back(pose);
			}

		
			float path_length = 0.0;
		
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
			
			geometry_msgs::PoseStamped last_pose;
			
			last_pose = *it;
			
			it++;
			
			for(; it != plan.end(); ++it){
				path_length += hypot( (*it).pose.position.x - last_pose.pose.position.x,
							(*it).pose.position.y - last_pose.pose.position.y)
				last_pose = *it;
			}
			std::cout << "Global path len: " << path_length << endl;
			
			return true;
		}
		
		else{
			ROS_WARN("Planner failed to find path");
			return false;
		}
	}
	else{
		ROS_WARN("Not valid start or goal");
		return false;
	}
}

void Example_Planner::getCoordinate(std::pair& coord){
	coord.first = coord.first - origin.first;
	coord.second = coord.second - origin.second;
}

int Example_Planner::convertToCellIndex(std::pair<float> coord){
	int cellIndx;
	
	cellIndx = getCellIndex(coord.first/resolution, coord.second/resolution);

	return cellIndx;
}

void Example_Planner::convertToCoordinate(int index, float& x, float& y){
	x = getCellColID(index) * resolution;
	y = getCellRowID(index) * resolution;

	x = x + originX;
	y = y + originY;
}

bool Example_Planner::isCellInsideMap(const float& x, const float& y){
	bool valid = true;
	
	if (x > (size.first * resolution) || y > (height * resolution))
		valid = false;
	return valid;
}

void Example_Planner::mapToWorld(const double& mx, const double& my, double& wx, double& wy){
	costmap2d::Costmap2D* costmap = costmap_ros_->getCostmap();

	wx = costmap->getOriginX() + mx * resolution;
	wy = costmap->getOriginY() + my * resolution;
}

std::vector<int> Example_Planner::AStarPlanner(const int& startCell, const int& goalCell){
	std::vector<int> bestPath;
	
	float g_score[mapSize];
	
	for (auto i = 0; i < mapSize; i++){
		g_score[i] = infinity;
		
	timespec time1, time2;
		
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	
	bestPath = findPath(startCell, goalCell, g_score);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
	
	std::cout << "Time for pathfinding algo: " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << std::endl;

	return bestPath;
}

std::vector<int> Example_Planner::findPath(int startCell, int goalCell, float g_score[]){
	value++;
	std::vector<int> bestPath;
	std::vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;
		
	g_score[startCell] = 0;
	CP.currentCell = startCell;
	CP.fcost = g_score[startCell] + H(startCell, goalCell);
	
	OPL.insert(CP);
	currentCell = startCell;

	while (!OPL.empty() && g_score[goalCell] == infinity){
		
		// choose cell with lowest cost
		currentCell = OPL.begin()->currentCell;
		
		// remove cur_cell from openlist
		OPL.erase(OPL.begin());
		
		std::vector<int> neighbourCells;
		neighbourCells = findFreeNeighbourCell(currentCell);
	
		for (auto i = 0; i < neighbourCells.size(); ++i){
			if (g_score[neighbourCells[i]] == infinity){
				g_score[neighbourCells[i]] = g_score[currentCell] + getMoveCost(currentCell, neighbourCells[i]);
				addNeighbourCellToOpenList(OPL, neighbourCells[i], goalCell, g_score);
			}
		}
	}
	
	if (g_score[goalCell] != infinity){
		bestPath = constructPath(startCell, goalCell, g_score);
		return bestPath;
	}
	else{
		std::cout << "Failed to find a path" << std::endl;
		return emptyPath;
	}
}

std::vector<int> Example_Planner::constructPath( const int& startCell, const int& goalCell, float g_score[]){
	std::vector<int> bestPath;
	std::vector<int> path;
		
	path.insert(path.begin() + bestPath.size(), goalCell);
	
	int currentCell = goalCell;
		
	while (currentCell != startCell){
		std::vector<int> neighbourCells;
		neighbourCells = findFreeNeighbourCell(currentCell);
		
		std::vector<float> gScoresNeighbours;
	
		for (auto i = 0; i < neighbourCells.size(); ++i)
			gScoresNeighbours.push_back(g_score[neighbourCells[i]]);
		
		int posMinGScore = distance(gScoresNeighbours.begin(), min_element(gScoresNeighbours.begin(), gScoresNeighbours.end()));

		currentCell = neighbourCells[posMinGScore];

		path.insert(path.begin() + path.size(), currentCell);
	}
	
	for (auto i = 0; i < path.size(); ++i)
		bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size()+1]);
	
	return bestPath;
}
		
float Example_Planner::H(const int& cellID, const int& goalCell){
	std::pair<int,int> coord_1 = std::make_pair(getCellRowID(goalCell), getCellColID(goalCell));
	std::pair<int,int> coord_2 = std::make_pair(getCellRowID(cellID), getCellColID(cellID));

	if (getNeighbourNumber() == 4)
		return min( abs(coord_1.first - coord_2.first), abs(coord_1.second - coord_2.second))*sqrt(2) + max(abs(coord_1.first - coord_2.first, coord_1.second - coord_2.second)) - min( abs(coord_1.first - coord_2.first), abs(coord_1.second - coord_2.second));
	
	else
		return abs(coord_1.first - coord_2.first) + abs(coord_1.second - coord_2.second);
}

void Example_Planner::addNeighbourCellToOpenList(multiset<cells>& OPL, int neighbourCell, int goalCell, int g_score[]){
	cells CP;
	CP.current_cell = neighbourCell;
	CP.fCost = g_score[neighbourCell] + H(neighbourCell, goalCell);
	OPL.insert(CP);
}

std::vector<int> Example_Planner::findFreeNeighbourCell(int CellID){
	int rowID = getCellRowID(CellID);
	int colID = getCellColID(CellID);
	int neighbourIndex;
	
	std::vector<int> freeNeighbourCells;
	
	for (auto i = -1; i <= 1; ++i){
		for (auto j = -1; j <= 1; ++j){
			 if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
				neighbourIndex = getCellIndex(rowID+i,colID+j);
        			if(isFree(neighbourIndex) )
	    				freeNeighbourCells.push_back(neighbourIndex);
			}
		}
	}

	return freeNeighbourCells;
}

