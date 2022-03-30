
#ifndef SEED_R7_NAVIGATION_H_
#define SEED_R7_NAVIGATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
  

class TurtlebotNavigationNode
{
 public:

   ros::Publisher pub_goal_;
   ros::Publisher map_initial_pub;
   ros::NodeHandle nh;
   void addGoal(const float x, const float y, const float yaw);
   bool popGoal();
   void mainloop();

 private:
  tf::TransformListener tfl_;
  tf::StampedTransform trans;

  geometry_msgs::PoseStamped current_goal_;
  std::list<geometry_msgs::PoseStamped> goals_;
  float x,y,yaw;

}; 
#endif   // TURTLEBOT3_DRIVE_H_

