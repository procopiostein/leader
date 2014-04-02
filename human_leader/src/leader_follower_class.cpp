#include "leader_follower_class.h"

#define DEG2RAD 0.01745

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
  return 0;
}

void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
  /// robot final goal (/robot_0/goal)
  robot_goal_x = msg->pose.position.x;
  robot_goal_y = msg->pose.position.y;
  robot_goal.header = msg->header;
  robot_goal.pose = msg->pose;
  resend_goal = true;
  get_plan();
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

void targets_callback(const mtt::TargetList& list){
 //where i update the list of candidates' position, etc
 
  for(uint i = 0; i < list.Targets.size(); i++){
    
    bool id_exists = false;
    std::vector<candidates_structPtr>::iterator it;
    
    for(it = candidates_list_class.begin(); it != candidates_list_class.end(); it++){
      if((uint)(*it)->id == (uint)list.Targets[i].id){
        id_exists = true;
        break;
      }
    }
    
    candidates_structPtr element;
    
    if(id_exists){
      element = *it;
    }
    else{
      element.reset(new candidates_struct_class);
      candidates_list_class.push_back(element);
    }

    element->id = list.Targets[i].id;
    element->pos_x = list.Targets[i].pose.position.x;
    element->pos_y = list.Targets[i].pose.position.y;
  } 
}

void leader_quality_callback(const visualization_msgs::MarkerArray &msg){
  //where i update the candidates list with the classification: good or bad leader
 
  //sweep my list 
  double vote = 0;
  std::vector<candidates_structPtr>::iterator it;
  for(it = candidates_list_class.begin(); it != candidates_list_class.end(); it++){

    for (uint i = 0; i < msg.markers.size(); i++){
      
      if((*it)->id == msg.markers[i].id){
        if(msg.markers[i].color.g == 1){
          vote = 0.01;
        }
        else{
          vote = -0.1;
        }
        break; //break when a match is found, no need to keep searching
      }
      else{
        //not present in last classification list
        vote = -0.1;
      }
    }
    
    double temp_score = (*it)->candidate_score + vote;
    if(temp_score < -0.1 || temp_score > 1){
//       ROS_INFO("loop score, c %d:%f, vote:%f",(*it)->id,(*it)->candidate_score, vote);
    }
    else{
      (*it)->candidate_score = temp_score;
//       ROS_INFO("loop score, c %d:%f, vote:%f",(*it)->id,(*it)->candidate_score, vote);
    }
  }
}

void leader_selection(){
  
  ///select best among good candidates (if any),
  ///according to some criteria (closest to robot)
  best_score = 0;
  navigation_mode = SOLO;
  
  follow_leader = false; //if good leader is present, this flag will change
  std::vector<candidates_structPtr>::iterator it; //iterator
//   curr_leader.reset(new candidates_struct_class); //reset curr_leader
  
  for(it = candidates_list_class.begin(); it != candidates_list_class.end(); it++){
    if((*it)->candidate_score > 0){ //threshold for being considered as good
      if((*it)->candidate_score > best_score){ //if better then previous best
        best_score = (*it)->candidate_score;
//         ROS_INFO("best score, c %d:%f",(*it)->id,best_score);
        
        //current leader is current iterator
        curr_leader = *it;
        follow_leader = true; //best leader
        resend_goal = true; //in the case leader is lost, algo can resend goal to riskrrt
        navigation_mode = LEADER;
        
        //advetise that leader has been found
        leader_found_flag.stamp = ros::Time::now();
        leader_found_flag.frame_id = "1";
        leaderfoundPublisher.publish(leader_found_flag);
      }
    }
  }
  
  /////////////////////////////////
  if(follow_leader){
    visualization_msgs::Marker marker;
      
    marker.scale.x = 1.2;
    marker.scale.y = 1.2;
    marker.scale.z = 1;
    marker.lifetime = ros::Duration(0.0);
    marker.id = 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = curr_leader->pos_x;
    marker.pose.position.y = curr_leader->pos_y;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leader_marker";
    marker.color.r = 0.0;
    marker.color.g = curr_leader->candidate_score;//0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::CUBE;

    marker_pub.publish(marker);       
    
    //create a point with leader position and transform to robot frame
    geometry_msgs::PointStamped map_point;
    map_point.header.frame_id = "map";
    map_point.header.stamp = ros::Time();
    map_point.point.x = curr_leader->pos_x;
    map_point.point.y = curr_leader->pos_y;
    map_point.point.z = 0.0;

    try{
      listener->transformPoint("base_link", map_point, base_point);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a point from \"map\" to \"base_link\": %s", ex.what());
    }
  }
  //////////////////////////////// 
  
/*  ///if leader found, check if new leader
  if(follow_leader){
    
    curr_leader->t_last_selection = ros::Time::now().toSec();
    
    if(curr_leader->id != prev_leader->id){

      //new leader found, make sure this is not temporary, (time hysteresis)
      ROS_INFO("curr_leader:%d, prev_leader:%d",curr_leader->id, prev_leader->id);
      
      double time_hysteresis = 
        curr_leader->t_last_selection - prev_leader->t_last_selection;
      
      ROS_INFO("time hysteresis:%.2f",time_hysteresis);
        
      if(time_hysteresis < 2.0){
        //keep old one
        curr_leader = prev_leader;
      }
      else{
        //enough time has passed to switch leaders
        prev_leader = curr_leader;
        
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
//       previous_leader = 20; //force reinitialization when leader is found again
      prev_leader.reset(new candidates_struct_class);
    }
  }*/
}

void leader_follow(){
/*  
  ///based on distance, change follow behavior (path or direct)
//   ROS_INFO("distance:%.2f",candidates_list[current_leader].dist_to_robot);
//   if(candidates_list[current_leader].dist_to_robot > 4){
  if(false){
    
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
//     pathPublisher.publish(leader_path);
    
    std_msgs::String leader_frame;        
    std::stringstream temp_str;
    temp_str << "/robot_" << current_leader << "/base_link";
    leader_frame.data = temp_str.str();
    
    ///get leader angle and distance from robot
    try{
      listener->lookupTransform("/base_link", temp_str.str(), ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }*/
    
//     ac->cancelAllGoals(); //cancel last goal sent to move_base
  
  
  switch(navigation_mode) {
    case SOLO :
      ROS_INFO("solo mode");
      //resend final goal to riskrrt
      if(resend_goal){
        //advetise that leader has been found
        leader_found_flag.stamp = ros::Time::now();
        leader_found_flag.frame_id = "0";
        leaderfoundPublisher.publish(leader_found_flag);
        
        next_pose = robot_goal;
        follow_pathPublisher.publish(next_pose); 
        resend_goal = false; //so it does not repeat
      }
      break;
      
    case LEADER :
      ROS_INFO("follow mode");
      double relative_angle  = atan2(base_point.point.y, base_point.point.x);
      double distance = sqrt(pow(base_point.point.x, 2) + pow(base_point.point.y, 2));

      double K_angular = 5.0;
      double K_linear = 1.0;
      
      geometry_msgs::Twist cmd_vel;
      cmd_vel.angular.z = relative_angle * K_angular;
      cmd_vel.linear.x = (distance - 2 ) * K_linear;
      if (cmd_vel.linear.x < 0) cmd_vel.linear.x = 0;
      if (cmd_vel.linear.x > 1.8) cmd_vel.linear.x = 1.8;
      
      if(follow_leader){
        robot_cmd_vel.publish(cmd_vel);
        ROS_INFO("cmd vel: %.2f",cmd_vel.linear.x);
      }
      else{
        ROS_INFO("cmd vel: 0");
      }
      break;
}
  
    
    
    
     
    
    //visualization
/*    geometry_msgs::PoseStamped visualization_pose;
    visualization_pose.pose.position.x = candidates_list[current_leader].pos_x;
    visualization_pose.pose.position.y = candidates_list[current_leader].pos_y;
    visualization_pose.pose.orientation = 
      tf::createQuaternionMsgFromYaw(candidates_list[current_leader].orientation);
    visualization_pose.header.stamp = ros::Time::now();
    follow_posePublisher.publish(visualization_pose); //goal passer
  }*/
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
  
  
  ///////////////////////////////////////////
  //move base action lib
  
//   MoveBaseClient _ac("/move_base", true);
//   ac = &_ac;
  
  

//    while(!ros::service::waitForService("/robot_0/move_base_node/make_plan", ros::Duration(3.0))) {
//       ROS_ERROR("Service move_base/make_plan not available - waiting.");
//    }
//    
//    client = n.serviceClient<nav_msgs::GetPlan>("/robot_0/move_base_node/make_plan");
//    if(!client) {
//       ROS_FATAL("Could not initialize get plan service from move_base_planner/make_plan (client name: %s)", client.getService().c_str());
//    }
//    ROS_INFO("Initialized Navstack Module.\n");
//    
//    while(!ac->waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }
   
   //////////////////////////////////////////
  
  //subscribers
//   dyn_objects_subscriber = n.subscribe("dynamic_objects", 100, &dyn_objects_callback);
//   leader_goal_subscriber = n.subscribe("leader_goal_pose",100,&leader_goal_callback);
  robot_goal_subscriber = n.subscribe("goal", 100, &robot_goal_callback);
  robot_pose_subscriber = n.subscribe("base_pose_ground_truth", 100, &robot_pose_callback);
  target_subscriber = n.subscribe("/targets", 100, &targets_callback);
  leader_class_subscriber = n.subscribe("/leader_quality", 100, &leader_quality_callback);
  
  //publishers //number 1 as final argument means queue size 1, so send the latest.
  pathPublisher = n.advertise<nav_msgs::Path>("leader_path", 1);
  robot_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  follow_pathPublisher = n.advertise<geometry_msgs::PoseStamped>("/leader_next_pose", 1);
  follow_posePublisher = n.advertise<geometry_msgs::PoseStamped>("anything", 1);
  leaderfoundPublisher = n.advertise<std_msgs::Header>("/leader_found", 1, true);
  marker_pub = n.advertise<visualization_msgs::Marker>("leader_markers", 1);
  
  
//   //initialize struct
//   for(int i = 0; i < 20; i++){//number of markers
//     candidates_list[i].goal_x = 99;
//     candidates_list[i].goal_y = 99;
//     candidates_list[i].good_candidate = true;
//     candidates_list[i].t_last_selection = 0;
//   }
  
  //initialize time counter for goal passed
//   t_wait = ros::Time::now().toSec();
//   double t_test = ros::Time::now().toSec();
  
  ros::Rate loop_rate(10);

 
  while (ros::ok()){
    
//     ///get robot heading
//     try{
//       listener->lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//     }
//     heading = tf::getYaw(transform.getRotation());
    
    /// leader selection heuristics
    leader_selection();
        
    /// leader follow management
    leader_follow();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
