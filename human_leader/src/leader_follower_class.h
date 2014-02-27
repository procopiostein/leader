#ifndef LEADER_FOLLOWER_CLASS_H
#define LEADER_FOLLOWER_CLASS_H

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
#include <visualization_msgs/MarkerArray.h>

#include "std_msgs/String.h"
#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"
#include "mtt/TargetList.h"

// #include "social_filter/TrajectoryObservation.h"
#include <boost/shared_ptr.hpp>

#define SOLO 1
#define PATH 2
#define LEADER 3

class candidates_struct_class{
  public:
    
    candidates_struct_class()
    {
      goal_x = 99;
      goal_y = 99;
      good_candidate = true;
      t_last_selection = 0;
      candidate_score = 0;
    }
    
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
    double candidate_score;
};

typedef boost::shared_ptr<candidates_struct_class> candidates_structPtr;

std::vector<candidates_structPtr> candidates_list_class;

// struct candidates_struct{
//   uint id;
//   double pos_x;
//   double pos_y;
//   double goal_x;
//   double goal_y;
//   double velocity;
//   double orientation;
//   double dist_to_goal;
//   double dist_btw_goals;
//   double dist_to_robot;
//   double min_dist_to_path;
//   double dist_to_robot_goal;
//   double t_last_selection;
//   bool good_candidate;
// };
// 
// candidates_struct candidates_list[20];

geometry_msgs::PointStamped base_point;
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
ros::Subscriber target_subscriber;
ros::Subscriber leader_class_subscriber;

ros::Publisher pathPublisher;
ros::Publisher robot_cmd_vel;
ros::Publisher follow_pathPublisher;
ros::Publisher follow_posePublisher;
ros::Publisher leaderfoundPublisher;
ros::Publisher marker_pub;

candidates_structPtr curr_leader;
candidates_structPtr prev_leader;

uint current_leader = 20; //must be different from any id at initialization
uint previous_leader = 20; //if outside limits of list size, will have error
double goals_distance = 99;
double distance_to_robot = 99;
double robot_dist_to_goal = 99;
double dist_best_cand = 99;
double dist_to_robot_goal = 99;
double dist_to_goal = 99;
double best_score = 0;
bool path_initialized = false;
bool send_goal = true;
bool follow_leader = false;
bool resend_goal = true;
bool follow_path = true;
uint navigation_mode = SOLO;
double ts_subgoal , tnow, tlast;
double t_goal, t_wait;
double robot_pose_x, robot_pose_y;
double robot_goal_x, robot_goal_y;
double heading;

void target_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
// void dyn_objects_callback(const social_filter::TrajectoryObservation::ConstPtr & dyn_objects);
void leader_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_pose_callback(const nav_msgs::Odometry::ConstPtr & msg);
void targets_callback(const mtt::TargetList& list);
void leader_quality_callback(const visualization_msgs::MarkerArray &msg);
int get_plan();
void leader_selection();
void leader_follow();

double euclidean_dist(double x1,double y1,double x2, double y2){
  double dist2 = ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
  return  (sqrt(dist2));
}



#endif // HUMAN_PROC_H

