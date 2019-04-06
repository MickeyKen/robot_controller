#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_robot_controller/WaypointMovingAction.h>

class WaypointMovingActionClient
{
public:
  WaypointMovingActionClient(std::string name, char *waypoint_file);
  ~WaypointMovingActionClient(void) { };
  void doneCb(const actionlib::SimpleClientGoalState& state,
  const my_robot_controller::WaypointMovingResultConstPtr& result);
  void activeCb();
  void feedbackCb(
    const my_robot_controller::WaypointMovingFeedbackConstPtr& feedback);
  void send_goals();

private:
  void read_waypoints(char *waypoint_file);
  actionlib::SimpleActionClient<my_robot_controller::WaypointMovingAction> ac;
  std::string action_name;
  my_robot_controller::WaypointMovingGoal goal;;

};

WaypointMovingActionClient::WaypointMovingActionClient(
  std::string name, char *waypoint_file) :
  ac("my_server", true) , action_name(name)
{
  ROS_INFO("%s Waiting For Server. . .", action_name.c_str());
  ac.waitForServer();

  ROS_INFO("%s Got a Server. . .", action_name.c_str());
  read_waypoints(waypoint_file);
  ROS_INFO("Sent Goal to Server. . .");
  send_goals();
}

void WaypointMovingActionClient::doneCb(
  const actionlib::SimpleClientGoalState& state,
  const my_robot_controller::WaypointMovingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result : %d", result->result);
  ros::shutdown();
}

void WaypointMovingActionClient::activeCb()
{
  ROS_INFO("Goal just went active. . .");
}

void WaypointMovingActionClient::feedbackCb(
  const my_robot_controller::WaypointMovingFeedbackConstPtr& feedback)
{
  ROS_INFO("Got feedback of Progress to Goal: %d", feedback->progress);
}

void WaypointMovingActionClient::send_goals()
{
  ac.sendGoal(goal, boost::bind(&WaypointMovingActionClient::doneCb,
  this, _1, _2),
  boost::bind(&WaypointMovingActionClient::activeCb, this),
  boost::bind(&WaypointMovingActionClient::feedbackCb, this, _1));
}

void WaypointMovingActionClient::read_waypoints(char *waypoint_file)
{
  FILE *fp;
  if((fp=fopen(waypoint_file,"r")) == NULL) {
    perror("file open");
    exit(1);
  }
  geometry_msgs::Pose p;
  while(!feof(fp)) {
    int ret=fscanf(fp, "%lf %lf %lf", &p.position.x,
    &p.position.y, &p.position.z);
    if(ret == EOF) break;
    goal.waypoint.push_back(p);
  }
  fclose(fp);
}

int main (int argc, char **argv)
{
  if (argc < 2) {
    printf("Usage: waypoint_move_client <waypoint file>\n");
    exit(1);
  }
  char *waypoint_file= argv[1];
  ros::init(argc, argv, "my_client");
  ROS_INFO("Waypoints file: %s, argc:%d", waypoint_file, argc);
  WaypointMovingActionClient client(ros::this_node::getName(),
  waypoint_file);
  ros::spin();
  return 0;
}
