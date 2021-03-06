#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <my_robot_controller/WaypointMovingAction.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>

class WaypointMovingActionServer
{
public:
  WaypointMovingActionServer(std::string name);
  ~WaypointMovingActionServer(void) { };
  void goalCB();
  void preemptCB();
  void controlCB(const turtlesim::Pose::ConstPtr& msg);

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
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

WaypointMovingActionServer::WaypointMovingActionServer(std::string name) :
 as_(nh_, name, false), action_name_(name)
{
  as_.registerGoalCallback(boost::bind(
    &WaypointMovingActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(
    &WaypointMovingActionServer::preemptCB, this));
  sub_ = nh_.subscribe("/turtle1/pose", 1,
    &WaypointMovingActionServer::controlCB, this);
  pub_ = nh_.advertise<geometry_msgs::Twist>(
    "/turtle1/cmd_vel", 1);
    as_.start();
}

void WaypointMovingActionServer::goalCB() {
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

void WaypointMovingActionServer::controlCB(
  const turtlesim::Pose::ConstPtr& msg)
{
  //printf("access");
  if (!as_.isActive() || as_.isPreemptRequested())
    return;
  if (progress_ < number_of_waypoints) {
    double l_scale = 4.0;
    double a_scale = 4.0;
    double error_tol = 0.01;

    if (start_) {
      start_x_ = msg->x;
      start_y_ = msg->y;
      start_theta_ = 0;
      start_ = false;

      geometry_msgs::Pose p = goal_.waypoint[progress_];
      len_ = fabs(sqrt((p.position.x - start_x_)*(p.position.x - start_x_)
        + (p.position.y - start_y_) * (p.position.y - start_y_)));
      angle_ = atan2((p.position.y - start_y_),(p.position.x - start_x_));
    }
    ROS_DEBUG("current position:(%.3f, %.3f) theta: %.3f",
      msg->x, msg->y, msg->theta);
    dis_error_ = len_ - fabs(sqrt((start_x_ - msg->x) * (start_x_ - msg->x)
    + (start_y_ - msg->y) * (start_y_ - msg->y)));
    theta_error_ = angle_ - (msg->theta - start_theta_);

    if (fabs(theta_error_) > error_tol) {
      command_.linear.x = 0;
      command_.angular.z = a_scale*theta_error_;
    } else if (dis_error_ > error_tol) {
      command_.linear.x = l_scale * dis_error_;
      command_.angular.z = 0;
    } else if (dis_error_ < error_tol && fabs(theta_error_ < error_tol)) {
      command_.linear.x = 0;
      command_.angular.z = 0;
      ROS_INFO("I'm getting to goal, %d/%d", progress_ + 1,
      number_of_waypoints);
      feedback_.progress = progress_;
      as_.publishFeedback(feedback_);

      start_ = true;
      progress_++;
    } else {
      command_.linear.x = l_scale * dis_error_;
      command_.angular.z = a_scale * theta_error_;
    }
    pub_.publish(command_);
  } else {
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    result_.result = progress_;
    as_.setSucceeded(result_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "act_server");
  WaypointMovingActionServer waypoint_moving(
    ros::this_node::getName());
  ros::spin();
  return 0;
}
