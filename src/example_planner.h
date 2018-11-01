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
  
  Example_Planner (ros::NodeHandle &); //this constructor is may be not needed
  Example_Planner ();
  Example_Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  
  ros::NodeHandle ROSNodeHandle;
  
  /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, 
		std::vector<geometry_msgs::PoseStamped>& plan
	       );
 

  void getCorrdinate (float& x, float& y);
  int convertToCellIndex (float x, float y);
  void convertToCoordinate(int index, float& x, float& y);
  bool isCellInsideMap(float x, float y);
  void mapToWorld(double mx, double my, double& wx, double& wy);
  vector<int> Example_Planner(int startCell, int goalCell);
  vector<int> findPath(int startCell, int goalCell, float g_score[]);
  vector<int> constructPath(int startCell, int goalCell, float g_score[]);
  float calculateHCost(int cellID, int goalCell){
  int x1=getCellRowID(goalCell);
  int y1=getCellColID(goalCell);
  int x2=getCellRowID(cellID);
  int y2=getCellColID(cellID);
    return abs(x1-x2)+abs(y1-y2);
	//return min(abs(x1-x2),abs(y1-y2))*sqrt(2) + max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2));
  }
  void addNeighborCellToOpenList(multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[]);
  vector <int> findFreeNeighborCell (int CellID);
  bool isStartAndGoalCellsValid(int startCell,int goalCell); 
  float getMoveCost(int CellID1, int CellID2);
  float getMoveCost(int i1, int j1, int i2, int j2);
  bool isFree(int CellID); //returns true if the cell is Free
  bool isFree(int i, int j); 

  int getCellIndex(int i,int j) //get the index of the cell to be used in Path
  {
   return (i*width)+j;  
  }
  int getCellRowID(int index)//get the row ID from cell index
  {
     return index/width;
  }
  int getCellColID(int index)//get colunm ID from cell index
  {
    return index%width;
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
