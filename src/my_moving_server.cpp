#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_robot_controller/WaypointMovingAction.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

class WaypointMovingActionServer
{
public:
  WaypointMovingActionServer(std::string name);
  ~WaypointMovingActionServer(void) { };
  void goalCB();
  void preemptCB();
  void executeCB(const my_robot_controller::WaypointMovingGoalConstPtr &goal);

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_robot_controller::WaypointMovingAction> as_;
  std::string action_name_;
  bool start_;
  int number_of_waypoints, progress_;
  double dis_error_, theta_error_;
  double start_x_, start_y_, start_theta_;
  double angle_, len_;
  my_robot_controller::WaypointMovingGoal goal_;
  my_robot_controller::WaypointMovingResult result_;
  my_robot_controller::WaypointMovingFeedback feedback_;
  geometry_msgs::Twist command_;
  ros::Publisher pub_;
};

WaypointMovingActionServer::WaypointMovingActionServer(std::string name) :
 as_(nh_, name, boost::bind(&WaypointMovingActionServer::executeCB, this, _1), false),
 action_name_(name)
{
  as_.registerGoalCallback(boost::bind(
    &WaypointMovingActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(
    &WaypointMovingActionServer::preemptCB, this));
  // as_.registerControlCallback(boost::bind(
    // &WaypointMovingActionServer::controlCB, this));
  pub_ = nh_.advertise<geometry_msgs::Twist>(
    "/cmd_vel", 1);
    as_.start();
}

void WaypointMovingActionServer::goalCB()
{
  goal_ = *as_.acceptNewGoal();
  number_of_waypoints = goal_.waypoint.size();
  progress_ = 0;
  start_ = true;
}

void WaypointMovingActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  result_.result = progress_;
  as_.setPreempted(result_, "I got Preempted!");
}

void WaypointMovingActionServer::executeCB(const my_robot_controller::WaypointMovingGoalConstPtr &goal)
{
  //printf("access");
  if (!as_.isActive() || as_.isPreemptRequested())
    return;
  if (progress_ < number_of_waypoints) {

    double l_scale = 4.0;
    double a_scale = 4.0;
    double error_tol = 0.01;

  } else {
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    result_.result = progress_;
    as_.setSucceeded(result_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_server");
  WaypointMovingActionServer waypoint_moving(
    ros::this_node::getName());
  ros::spin();
  return 0;
}
