#pragma once

/* STD CPP DEPS*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>
#include <utility>

/* ROS LIBS*/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

/* ROS PLANNER DEPS*/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


/**
 * @struct cells
 * @brief A struct that represents a cell and its fCost.
 */
struct cells {
	int currentCell;
	float fCost;

};      

namespace example_planner {
  
class Example_Planner : public nav_core::BaseGlobalPlanner {
public:
  
  Example_Planner (const ros::NodeHandle &); //this constructor is may be not needed
  Example_Planner ();
  Example_Planner(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros);
  
  ros::NodeHandle NodeHandler;
  
  /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, 
		std::vector<geometry_msgs::PoseStamped>& plan
	       );
 

  void getCoordinate (std::pair<float, float>& coord);
  int convertToCellIndex (std::pair<float, float> coord);
  void convertToCoordinate(int index, float& x, float& y);
  bool isCellInsideMap(const float& x,const float& y);
  void mapToWorld(const double& mx,const double& my,double& wx, double& wy);
  std::vector<int> AStarPlanner(const int& startCell,const int& goalCell);
  std::vector<int> findPath(int startCell, int goalCell, float g_score[]);
  std::vector<int> constructPath(const int& startCell,const int& goalCell, float g_score[]);
  float H(const int& cellID,const int& goalCell){
  int x1=getCellRowID(goalCell);
  int y1=getCellColID(goalCell);
  int x2=getCellRowID(cellID);
  int y2=getCellColID(cellID);
    return abs(x1-x2)+abs(y1-y2);
	//return min(abs(x1-x2),abs(y1-y2))*sqrt(2) + max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2));
  }
  void addNeighbourCellToOpenList(std::multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[]);
  std::vector <int> findFreeNeighbourCell (int CellID);
  bool isStartAndGoalCellsValid(const int& startCell,const int& goalCell); 
  float getMoveCost(const int& CellID1,const int& CellID2);
  float getMoveCost(const int& i1,const int& j1,const int& i2,const int& j2);
  bool isFree(const int& CellID); //returns true if the cell is Free
  bool isFree(const int& i,const int& j); 

  int getCellIndex(int i,int j) //get the index of the cell to be used in Path
  {
   return (i*size.first)+j;  
  }
  int getCellRowID(int index)//get the row ID from cell index
  {
    return index/size.first;
  }
  int getCellColID(int index)//get colunm ID from cell index
  {
    return index%size.first;
  }

  std::pair <float, float> origin;

  float resolution;
  costmap_2d::Costmap2DROS* costmap_ros_;
  double step_size_, min_dist_from_robot_;
  costmap_2d::Costmap2D* costmap_;
  //base_local_planner::WorldModel* world_model_;
  bool initialized_;
  std::pair <int, int> size;
  unsigned int cost;

};

};
