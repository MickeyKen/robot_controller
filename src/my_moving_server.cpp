#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_robot_controller/WaypointMovingAction.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class WaypointMovingActionServer
{
public:
  WaypointMovingActionServer(std::string name);
  ~WaypointMovingActionServer(void) { };
  void goalCB();
  void preemptCB();
  void executeCB(const my_robot_controller::WaypointMovingGoalConstPtr &goal_);

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_robot_controller::WaypointMovingAction> as_;
  std::string action_name_;
  std::string base_frame;
  std::string child_frame;
  bool start_;
  int number_of_waypoints, progress_;
  double dis_error_, theta_error_;
  double start_x_, start_y_, start_theta_;
  double angle_, len_;
  double x_speed;
  double y_speed;
  double ang_speed;
  int rate;
  my_robot_controller::WaypointMovingGoal goal_;
  my_robot_controller::WaypointMovingResult result_;
  my_robot_controller::WaypointMovingFeedback feedback_;
  geometry_msgs::Twist command_;
  ros::Publisher pub_;
  tf::TransformListener listener;
  tf::StampedTransform transform;
};

WaypointMovingActionServer::WaypointMovingActionServer(std::string name) :
 as_(nh_, name, boost::bind(&WaypointMovingActionServer::executeCB, this, _1), false),
 action_name_(name)
{
  as_.registerGoalCallback(boost::bind(
    &WaypointMovingActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(
    &WaypointMovingActionServer::preemptCB, this));
  // as_.registerExecuteCallback(boost::bind(
  //   &WaypointMovingActionServer::executeCB, this));
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

    geometry_msgs::Pose p = goal_.waypoint[progress_];
    double target_x = p.position.x;
    double target_y = p.position.y;

    tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double target_theta = yaw;

    double current_x = 0.0;
    double current_y = 0.0;

    rate = 20;
    x_speed = 0.4;
    y_speed = 0.4;

    ros::Rate r(rate);

    base_frame = "/base_footprint";
    child_frame = "/odom";

    if (target_x == target_y) {
      if (target_x < 0) {
        command_.linear.x = - (x_speed);
      } else {
        command_.linear.x = x_speed;
      }
      if (target_y < 0) {
        command_.linear.y = - (y_speed);
      } else {
        command_.linear.y = y_speed;
      }

      while (current_x < target_x && current_y < target_y && nh_.ok()) {
        pub_.publish(command_);
        r.sleep();
        try {
          listener.waitForTransform(base_frame, child_frame,
            ros::Time(0), ros::Duration(1.0));
          listener.lookupTransform(base_frame, child_frame,
            ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
        current_x += fabs(transform.getOrigin().x());
        current_y += fabs(transform.getOrigin().y());

      }

    } else if (target_x > target_y) {

    } else {

    }
    command_.linear.x = 0;
    command_.linear.y = 0;
    pub_.publish(command_);
    ROS_INFO("I'm getting to goal, %d/%d", progress_ + 1,
    number_of_waypoints);
    feedback_.progress = progress_;
    as_.publishFeedback(feedback_);

    start_ = true;
    progress_++;

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
