#ifndef SEED_R7_CMD_VDL_H_
#define SEED_R7_CMD_VDL_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)



#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

class Mover3Drive
{
 public:
  Mover3Drive();
  ~Mover3Drive();
  bool init();
// bool  controlLoop(const ros::TimerEvent& e );
 bool  controlLoop();
 bool  controlLoop2();

 bool  controlLoop3();
 bool  stop();
  ros::Publisher cmd_vel_pub_;
  ros::NodeHandle nh;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;



  // Function prototypes

  void updatecommandVelocity(double linear_x,double linear_y,double linear_Z, double angular_x,double angular_y,double angular_z);


//  void controlLoop(const ros::TimerEvent& e );

};

#endif // SEED_R7_CMD_VEL_H_


