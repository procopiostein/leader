#ifndef LEADER_FOLLOWER_H
#define LEADER_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
// #include "human_leader/humanPose.h" //this is for the msg defined
// #include "human_leader/humanPoses.h"

#include "human_leader/TrajectoryObservation.h"

struct candidates_struct{
  int id;
  double pos_x;
  double pos_y;
  double goal_x;
  double goal_y;
  double velocity;
  double orientation;
  double dist_to_goal;
  double dist_btw_goals;
  double dist_to_robot;
  double min_dist_to_path;
  double dist_to_robot_goal;
  double t_last_selection;
  bool good_candidate;
};

candidates_struct candidates_list[20];

geometry_msgs::PoseStamped visualization_pose;
geometry_msgs::PoseStamped leader_pose;
geometry_msgs::PoseStamped next_pose;
geometry_msgs::PoseStamped robot_goal;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
geometry_msgs::PoseStamped start_pose, goal_pose, map_goal_pose;
move_base_msgs::MoveBaseGoal goal;
nav_msgs::Path leader_path;
geometry_msgs::Twist cmd_vel;
ros::ServiceClient client;
nav_msgs::GetPlan srv;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std_msgs::Header leader_found_flag;

tf::TransformListener* listener = NULL;
tf::StampedTransform transform;

MoveBaseClient* ac = NULL;

ros::Subscriber dyn_objects_subscriber;
ros::Subscriber leader_goal_subscriber;
ros::Subscriber robot_goal_subscriber;
ros::Subscriber robot_pose_subscriber;

ros::Publisher pathPublisher;
ros::Publisher robot_cmd_vel;
ros::Publisher follow_pathPublisher;
ros::Publisher follow_posePublisher;
ros::Publisher leaderfoundPublisher;
ros::Publisher marker_pub;

uint current_leader = 20; //must be different from any id at initialization
uint previous_leader = 20; //if outside limits of list size, will have error
double goals_distance = 99;
double distance_to_robot = 99;
double robot_dist_to_goal = 99;
double dist_best_cand = 99;
double dist_to_robot_goal = 99;
double dist_to_goal = 99;
bool path_initialized = false;
bool send_goal = true;
bool follow_leader = false;
bool resend_goal = true;
bool follow_path = true;
double ts_subgoal , tnow, tlast;
double t_goal, t_wait;
double robot_pose_x, robot_pose_y;
double robot_goal_x, robot_goal_y;
double heading;

void target_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
void dyn_objects_callback(const human_leader::TrajectoryObservation::ConstPtr & dyn_objects);
void leader_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_pose_callback(const nav_msgs::Odometry::ConstPtr & msg);
int get_plan();
void leader_selection();
void leader_follow();

double euclidean_dist(double x1,double y1,double x2, double y2){
  double dist2 = ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
  return  (sqrt(dist2));
}



#endif // HUMAN_PROC_H

