#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>

class Mover3Drive{
	private:
		ros::NodeHandle nh;
	public:
    Mover3Drive();
   ~Mover3Drive();
    void init();
    ros::Publisher cmd_vel_pub_;
    void  controlLoop();
    void  stop();
    void updatecommandVelocity(double linear_x,double linear_y,double linear_Z, double angular_x,double angular_y,double angular_z);
};

Mover3Drive::Mover3Drive()
{

      
 //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");	//Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
}

Mover3Drive::~Mover3Drive()
{
  updatecommandVelocity(0.0, 0.0, 0.0, 0.0, 0.0 ,0.0);
  ros::shutdown();
}



void Mover3Drive::controlLoop()
{
  updatecommandVelocity(0.1,0,0,0,0,0);
}

void Mover3Drive::stop()
{
  updatecommandVelocity(0.0,0,0,0,0,0);
}
void Mover3Drive::updatecommandVelocity(double linear_x,double linear_y,double linear_z, double angular_x,double angular_y,double angular_z)
{
  geometry_msgs::Twist cmd_vel;
  
  cmd_vel.linear.x  = linear_x;
  cmd_vel.linear.y  = linear_y;
  cmd_vel.linear.z  = linear_z;
  cmd_vel.angular.x= angular_x;
  cmd_vel.angular.y= angular_y;
  cmd_vel.angular.z= angular_z;
  cmd_vel_pub_.publish(cmd_vel);

}

int main(int argc, char* argv[])

{
   ros::init(argc, argv, "tb3_0", ros::init_options::AnonymousName);
   ros::NodeHandle nh;
   ros::Publisher cmd_vel_pub_;
   geometry_msgs::Twist vel;
   

   Mover3Drive Mover_drive;
   ros::Rate loop_rate(100);
   ros::Time start=ros::Time::now();
  

  while (ros::ok())
 {
   ros::spinOnce();
   
   ros::Time now=ros::Time::now(); 
  if(ros::Duration(5.0)>now-start)
  {
    Mover_drive.stop();
    ROS_INFO("aaa");
	  ///Mover_drive.controlLoop();
  }

 }
   return 0;
}