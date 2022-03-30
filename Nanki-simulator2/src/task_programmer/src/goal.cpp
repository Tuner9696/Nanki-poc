#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class RSJRobotTestNode
{
private:
  ros::Publisher pub_goal_;
  tf::TransformListener tfl_;

  std::list<geometry_msgs::PoseStamped> goals_;
  geometry_msgs::PoseStamped current_goal_;

public:
  RSJRobotTestNode()
  {
    ros::NodeHandle nh;
    pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 5, true);
  }
  void addGoal(const float x, const float y, const float yaw)
  {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    goals_.push_back(goal);
  }
  bool popGoal()
  {
    if (goals_.size() == 0) return false;

    current_goal_ = goals_.front();
    goals_.pop_front();
    ROS_INFO("Applying goal %0.3f %0.3f %0.3f",
        current_goal_.pose.position.x,
        current_goal_.pose.position.y,
        tf::getYaw(current_goal_.pose.orientation));
    pub_goal_.publish(current_goal_);

    return true;
  }
  void mainloop()
  {
    ROS_INFO("Hello ROS World!");

    if (!popGoal())
    {
      ROS_ERROR("No goal specified");
      return;
    }

    ros::Rate rate(10.0);
    while (ros::ok())
    {
      rate.sleep();
      ros::spinOnce();

      float x, y, yaw;
      try
      {
        tf::StampedTransform trans;
        tfl_.waitForTransform("map", "base_link", 
            ros::Time(0), ros::Duration(0.5));
        tfl_.lookupTransform("map", "base_link", 
            ros::Time(0), trans);
        x = trans.getOrigin().x();
        y = trans.getOrigin().y();
        yaw = tf::getYaw(trans.getRotation());
      }
      catch(tf::TransformException &e)
      {
        ROS_WARN("%s", e.what());
        continue;
      }

      float yaw_goal = tf::getYaw(current_goal_.pose.orientation);
      float yaw_error = yaw - yaw_goal;
      if (yaw > M_PI) yaw -= 2.0 * M_PI;
      else if (yaw < -M_PI) yaw += 2.0 * M_PI;

      if (hypotf(x - current_goal_.pose.position.x,
                 y - current_goal_.pose.position.y) < 0.15 &&
          fabs(yaw_error) < 0.3)
      {
        if (!popGoal())
        {
          ROS_INFO("Finished");
          return;
        }
        ROS_INFO("Next goal applied");
      }
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rsj_robot_test_node");

  RSJRobotTestNode robot_test;

  // 行き先を追加
  robot_test.addGoal(0.3, -0.3, 0.0);
  robot_test.addGoal(0.2, 0.2, 1.57);

  robot_test.mainloop();
}