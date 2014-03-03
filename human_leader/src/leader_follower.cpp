#include "ros/ros.h"
#include "tf/tf.h"
#include "leader_follower.h"

#define DEG2RAD 0.01745



/*******
this node manages the choice of a leader and how the robot should follow it

it requires ghmm to work, as that node will publish goal predictions. this program
evaluates each goal in comparison to the robot goal, then when a leader is foun 
(by goal similarity), it subscribes to the chosen leader position (dynamic_objects)
that is being published normally by ar_human_proc, which wraps markers into that structure
********/

void dyn_objects_callback(const human_leader::TrajectoryObservation::ConstPtr & dyn_objects){

  ///update candidtes info and store path of leader if one has been found
  
  //update candidates information
  candidates_list[dyn_objects->object_id].pos_x = dyn_objects->pose.x;
  candidates_list[dyn_objects->object_id].pos_y = dyn_objects->pose.y;
  candidates_list[dyn_objects->object_id].velocity = 
    hypot(dyn_objects->velocity.linear.x, dyn_objects->velocity.linear.y);
  
  /// not being used for now  
  //measure distance from robot precomputed path
  double min_distance_to_path = 99;
  if(srv.response.plan.poses.size() != 0){
    std::vector<geometry_msgs::PoseStamped>::iterator it = srv.response.plan.poses.begin();
    for (; it!=srv.response.plan.poses.end(); ++it) {
      double current_distance = euclidean_dist((*it).pose.position.x,(*it).pose.position.y,
                    dyn_objects->pose.x,dyn_objects->pose.y);
      if(current_distance < min_distance_to_path) min_distance_to_path = current_distance;
    }
  }
  candidates_list[dyn_objects->object_id].min_dist_to_path = min_distance_to_path;
  //////////////////////////////////////////////
  
  /// record leader path if leader has been found
  if(current_leader == dyn_objects->object_id && follow_leader){
    if(leader_path.poses.size() > 0){
      tlast = leader_path.poses.back().header.stamp.toSec();
      tnow = ros::Time::now().toSec();
      ts_subgoal = tnow - tlast;
      ///store patsend_goalh according to time step
      if(ts_subgoal > 0.5){
        leader_pose.pose.position.x = dyn_objects->pose.x;
        leader_pose.pose.position.y = dyn_objects->pose.y;
        leader_pose.pose.orientation = tf::createQuaternionMsgFromYaw(dyn_objects->pose.theta /*+ 1.57*/);
        leader_pose.header.stamp = ros::Time::now();
        leader_path.poses.push_back(leader_pose);
      }
    }
    else{
      ///only executes on first message
      path_initialized = true;
      leader_pose.pose.position.x = dyn_objects->pose.x;
      leader_pose.pose.position.y = dyn_objects->pose.y;
      leader_pose.pose.orientation = tf::createQuaternionMsgFromYaw(dyn_objects->pose.theta /*+ 1.57*/);
      leader_pose.header.stamp = ros::Time::now();
      leader_path.poses.push_back(leader_pose);
    }
//     ROS_INFO("published pose"); 
    leader_pose.header.frame_id = "/map";
    follow_posePublisher.publish(leader_pose); //goal passer
    
    
    /////////////////////////////////
    visualization_msgs::Marker marker;
      
    marker.scale.x = 1.2;
    marker.scale.y = 1.2;
    marker.scale.z = 1;
    marker.lifetime = ros::Duration(1.0);
    marker.id = 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = leader_pose.pose;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leader_marker";
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    if(follow_path){
      marker.type = visualization_msgs::Marker::CUBE;
    }
    else{
      marker.type = visualization_msgs::Marker::CYLINDER;
    }
      
    marker_pub.publish(marker);
    ////////////////////////////////    
  }
}

void leader_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
  /// receive leader goal, can receive from predictors
  /// but now it is from published goal topic sent by myself
  int temp_id = msg->pose.position.z;
  candidates_list[temp_id].goal_x = msg->pose.position.x;
  candidates_list[temp_id].goal_y = msg->pose.position.y;
}

int get_plan(){
  /// generate base plan of robot to be used in leader selection
  ROS_INFO("entered get_plan");

  start_pose.header.frame_id = "/map"; //force change, because tf is unitary
  
  ROS_INFO("start: %.2f %.2f", start_pose.pose.position.x, start_pose.pose.position.y);
  ROS_INFO("goal: %.2f %.2f", goal_pose.pose.position.x, goal_pose.pose.position.y);
  std::cout << "start pose frame_id:" << start_pose.header.frame_id << std::endl;
  std::cout << "map goal frame_id:" << goal_pose.header.frame_id << std::endl;
  
  // set pose
  srv.request.start = start_pose;
  srv.request.goal = robot_goal;
  ROS_INFO("calling planner ...");
  if (client.call(srv)){
    if (srv.response.plan.poses.size() == 0) {
      ROS_INFO("... no plan found!");
      return 1;
    }
  }
  else{
    ROS_ERROR("failed to call service make_plan");
    return 1;
  }
}

void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
  /// robot final goal (/robot_0/goal)
  robot_goal_x = msg->pose.position.x;
  robot_goal_y = msg->pose.position.y;
  robot_goal.header = msg->header;
  robot_goal.pose = msg->pose;
  resend_goal = true;
//   get_plan();
}

void robot_pose_callback(const nav_msgs::Odometry::ConstPtr & msg){
  /// update robot's pose
  robot_pose_x = -msg->pose.pose.position.y;
  robot_pose_y = msg->pose.pose.position.x; 
  robot_dist_to_goal = euclidean_dist(
    robot_pose_x, robot_pose_y, robot_goal_x, robot_goal_y);
  
  /// used to generate base plan
  start_pose.header = msg->header;
  start_pose.pose = msg->pose.pose;
  start_pose.pose.position.x = robot_pose_x;
  start_pose.pose.position.y = robot_pose_y;
}

void leader_selection(){
  
  ///sweeps list to find good candidates
  dist_best_cand = 99; //reset closest distance
  for(int i = 0; i < 20; i++){
    
    ///compute candidate information
   
    goals_distance = hypot(
      candidates_list[i].goal_x-robot_goal_x,candidates_list[i].goal_y - robot_goal_y);
    
    distance_to_robot = hypot(
      candidates_list[i].pos_x - robot_pose_x, candidates_list[i].pos_y - robot_pose_y);
    
    dist_to_robot_goal = hypot(
      candidates_list[i].pos_x - robot_goal_x, candidates_list[i].pos_y - robot_goal_y);
    
    dist_to_goal = hypot(candidates_list[i].goal_x-candidates_list[i].pos_x,
      candidates_list[i].goal_y-candidates_list[i].pos_y );
    
    candidates_list[i].dist_btw_goals = goals_distance;
    candidates_list[i].dist_to_robot = distance_to_robot;
    candidates_list[i].dist_to_robot_goal = dist_to_robot_goal;
    candidates_list[i].dist_to_goal = dist_to_goal; //not used
    
//     ROS_INFO("cand id:%d, goals dist:%f, goals dist2:%f", i, goals_distance, goals_distance2);
    
    ///leader selection criteria, if not satisfied, 
    ///candidate is marked as bad candicate (good_candidate=false)
    //goals are less then 5 meters appart 
    //leader is less then 8 m from robot
    //leader is closer to robot's goal then the robot
    //leader velocity is higher than 0.5m/s
//     if(goals_distance < 5.0 && distance_to_robot < 8.0 && 
//       (dist_to_robot_goal < robot_dist_to_goal) && candidates_list[i].velocity > 0.5 &&
//        dist_to_goal > 1)
    
    if(goals_distance < 5.0 && distance_to_robot < 8.0 && 
      (dist_to_robot_goal < robot_dist_to_goal) && candidates_list[i].velocity > 0.5){
      
      ROS_INFO("good cand:%d - gd:%.2f, d_2_r:%.2f, cd_2_rg:%.2f, rd_2_rg:%.2f, vel:%.2f",
        i, goals_distance, distance_to_robot, 
        dist_to_robot_goal, robot_dist_to_goal, candidates_list[i].velocity);
      candidates_list[i].good_candidate = true;
    }
    else{
      ROS_INFO("bad cand:%d - gd:%.2f, d_2_r:%.2f, cd_2_rg:%.2f, rd_2_rg:%.2f, vel:%.2f",
        i, goals_distance, distance_to_robot, 
        dist_to_robot_goal, robot_dist_to_goal, candidates_list[i].velocity);
      candidates_list[i].good_candidate = false;
    }    
  }
  ///////// finished good candidates sweep ////////////////
  
  
//   ///select best of good candidates (if any),
//   ///according to some criteria (closest to robot)
//   follow_leader = false; //if good leader is present, this flag will change
//   for(int i = 0; i < 20; i++){
//     if(candidates_list[i].good_candidate){
//       if(candidates_list[i].dist_to_robot_goal < dist_best_cand){
//         dist_best_cand = candidates_list[i].dist_to_robot;
//         current_leader = i;
//         follow_leader = true;
//         //send message to riskrrt to stop planning (leader has been found)
//         leader_found_flag.stamp = ros::Time::now();
//         leader_found_flag.frame_id = "1";
//         leaderfoundPublisher.publish(leader_found_flag);
//       }
//     }
//   }

  ///select best of good candidates (if any),
  ///according to some criteria (closest to robot)
  follow_leader = false; //if good leader is present, this flag will change
  for(int i = 0; i < 20; i++){
    if(candidates_list[i].good_candidate){
      if(candidates_list[i].dist_to_robot < dist_best_cand){
        dist_best_cand = candidates_list[i].dist_to_robot;
        current_leader = i;
        follow_leader = true;
        //send message to riskrrt to stop planning (leader has been found)
        leader_found_flag.stamp = ros::Time::now();
        leader_found_flag.frame_id = "1";
        leaderfoundPublisher.publish(leader_found_flag);
      }
    }
  }
  
//   ///if leader found, check if new leader
//   if(follow_leader){
//     if(current_leader != previous_leader){
// //       ROS_INFO("found new leader, id:%d, reseting path",current_leader);
//       previous_leader = current_leader;
//       
//       resend_goal = true; //ready to resend original goal, if leader is lost
//       send_goal = true; //ready to publish subgoal
//       
//       //reset recorded path
//       path_initialized = false;
//       leader_path.header.stamp = ros::Time::now();
//       leader_path.header.frame_id = "/map";
//       leader_path.poses.clear();
//     }
//   }
  
  
    ///if leader found, check if new leader
  if(follow_leader){
    
    candidates_list[current_leader].t_last_selection = ros::Time::now().toSec();
    
    if(current_leader != previous_leader){

      //new leader found, make sure this is not temporary, (time hysteresis)
      
      ROS_INFO("curr_lead:%d, prev_lead:%d",current_leader, previous_leader);
      
      double time_hysteresis = 
        candidates_list[current_leader].t_last_selection -
        candidates_list[previous_leader].t_last_selection;
      
      ROS_INFO("time hysteresis:%.2f",time_hysteresis);
        
      if(time_hysteresis < 2.0){
        
        //keep old one
        current_leader = previous_leader;
      }
      else{
        //enough time has passed to switch leaders
        previous_leader = current_leader;
        
        resend_goal = true; //ready to resend original goal, if leader is lost
        send_goal = true; //ready to publish subgoal
        
        //reset recorded path
        path_initialized = false;
        leader_path.header.stamp = ros::Time::now();
        leader_path.header.frame_id = "/map";
        leader_path.poses.clear();
      }
    }
  }
  
  
  ///no leader has been found
  else{
    //avoid constant resend
    if(resend_goal){
//       ROS_INFO("resending original goal");
      ac->cancelAllGoals(); //cancel last goal sent to move_base
      //send message to riskrrt to plan again
      leader_found_flag.stamp = ros::Time::now();
      leader_found_flag.frame_id = "0";
      leaderfoundPublisher.publish(leader_found_flag);
      
      //resend final goal to riskrrt
      next_pose = robot_goal;
      follow_pathPublisher.publish(next_pose); 
      resend_goal = false; //so it does not repeat
      send_goal = false; //disable leader following
      previous_leader = 20; //force reinitialization when leader is found again
    }
  }
}

void leader_follow(){
  
  ///based on distance, change follow behavior (path or direct)
//   ROS_INFO("distance:%.2f",candidates_list[current_leader].dist_to_robot);
  if(candidates_list[current_leader].dist_to_robot > 4){
    
    ///follow path
//     ROS_INFO("following path"); 
    follow_path = true;
    if(path_initialized){
    
      ///if ready to send next subgoal
      if(send_goal){

        ///follow path : set subgoal as first path position as goal
        next_pose.header.frame_id = "/map";
        next_pose.header.stamp = ros::Time::now();
        next_pose.pose.position.x = leader_path.poses[0].pose.position.x;
        next_pose.pose.position.y = leader_path.poses[0].pose.position.y;
        next_pose.pose.orientation = leader_path.poses[0].pose.orientation;
                
//         follow_pathPublisher.publish(next_pose); //uses riskrrt
        follow_posePublisher.publish(next_pose); //goal passer
        
        /// uses move_base action client
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = leader_path.poses[0].pose.position.x;
        goal.target_pose.pose.position.y = leader_path.poses[0].pose.position.y;
        goal.target_pose.pose.orientation = leader_path.poses[0].pose.orientation;
        
        ac->sendGoal(goal);
        send_goal = false;
      }
      
      else{
        
        ///manage subgoal: pass goal based on distance from next_pose
        ROS_DEBUG("passing with distance"); 
        double distance_to_pose = hypot(
          robot_pose_x - next_pose.pose.position.x,
          robot_pose_y - next_pose.pose.position.y);
        
        //closer than 2 meters from subgoal, can pass next
        if(distance_to_pose < 2.0){
          if(leader_path.poses.size() > 1){
            leader_path.poses.erase(leader_path.poses.begin());
            //ready to send next subgoal
            send_goal = true;
          }
        }
      }
      pathPublisher.publish(leader_path);
    }
  }
  else{
    ///direct follow
//     ROS_INFO("following direct");
    follow_path = false;
    ac->cancelAllGoals(); //cancel last goal sent to move_base
    leader_path.poses.clear();
    pathPublisher.publish(leader_path);
    
    std_msgs::String leader_frame;        
    std::stringstream temp_str;
    temp_str << "/robot_" << current_leader << "/base_link";
    leader_frame.data = temp_str.str();
    
    ///get leader angle and distance from robot
    try{
      listener->lookupTransform("/robot_0/base_link", temp_str.str(), ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    double relative_angle  = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    double distance = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

    double K_angular = 5.0;
    double K_linear = 1.0;
    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = relative_angle * K_angular;
    cmd_vel.linear.x = (distance - 2 ) * K_linear;
    if (cmd_vel.linear.x < 0) cmd_vel.linear.x = 0;
//     ROS_INFO("cmd vel: %.2f",cmd_vel.linear.x);
    robot_cmd_vel.publish(cmd_vel);
    
    //visualization
//     geometry_msgs::PoseStamped visualization_pose;
//     visualization_pose.pose.position.x = candidates_list[current_leader].pos_x;
//     visualization_pose.pose.position.y = candidates_list[current_leader].pos_y;
//     visualization_pose.pose.orientation = 
//       tf::createQuaternionMsgFromYaw(candidates_list[current_leader].orientation);
//     visualization_pose.header.stamp = ros::Time::now();
//     follow_posePublisher.publish(visualization_pose); //goal passer
  }
}

int main(int argc, char **argv){ 
  ros::init(argc, argv, "leader_follower");
  ros::NodeHandle n;

  //initialize leader_path
  leader_path.header.stamp = ros::Time::now();
  leader_path.header.frame_id = "/map";
  
  //used in transformations
  tf::TransformListener _listener;
  listener = &_listener;
  
  //move base action lib
  MoveBaseClient _ac("/robot_0/move_base", true);
  ac = &_ac;
  
  ///////////////////////////////////////////

   while(!ros::service::waitForService("/robot_0/move_base_node/make_plan", ros::Duration(3.0))) {
      ROS_ERROR("Service move_base/make_plan not available - waiting.");
   }
   
   client = n.serviceClient<nav_msgs::GetPlan>("/robot_0/move_base_node/make_plan");
   if(!client) {
      ROS_FATAL("Could not initialize get plan service from move_base_planner/make_plan (client name: %s)", client.getService().c_str());
   }
   ROS_INFO("Initialized Navstack Module.\n");
   
   while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
   
   //////////////////////////////////////////
  
  //subscribers
  dyn_objects_subscriber = n.subscribe("dynamic_objects", 100, &dyn_objects_callback);
  leader_goal_subscriber = n.subscribe("leader_goal_pose",100,&leader_goal_callback);
  robot_goal_subscriber = n.subscribe("/robot_0/goal", 100, &robot_goal_callback);
  robot_pose_subscriber = n.subscribe("/robot_0/base_pose_ground_truth", 100, &robot_pose_callback);
  
  //publishers //number 1 as final argument means queue size 1, so send the latest.
  pathPublisher = n.advertise<nav_msgs::Path>("leader_path", 1);
  robot_cmd_vel = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1);
  follow_pathPublisher = n.advertise<geometry_msgs::PoseStamped>("/robot_0/leader_next_pose", 1);
  follow_posePublisher = n.advertise<geometry_msgs::PoseStamped>("anything", 1);
  leaderfoundPublisher = n.advertise<std_msgs::Header>("/robot_0/leader_found", 1, true);
  marker_pub = n.advertise<visualization_msgs::Marker>("leader_markers", 1);
  
  
  //initialize struct
  for(int i = 0; i < 20; i++){//number of markers
    candidates_list[i].goal_x = 99;
    candidates_list[i].goal_y = 99;
    candidates_list[i].good_candidate = true;
    candidates_list[i].t_last_selection = 0;
  }
  
  //initialize time counter for goal passed
  t_wait = ros::Time::now().toSec();
  double t_test = ros::Time::now().toSec();
  
  ros::Rate loop_rate(100);

 
  while (ros::ok()){
    
    ///get robot heading
    try{
      listener->lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    heading = tf::getYaw(transform.getRotation());
    
    /// leader selection heuristics
    leader_selection();
        
    /// leader follow management
    if(follow_leader) leader_follow();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
