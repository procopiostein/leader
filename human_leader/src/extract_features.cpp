/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/*! \brief Code for feature extraction
 *
 *  extract features from targets received from MTT wrt the robot
 *  the features are printed in the command line and should be stored
 *  in a file (txt) for further processing with matlab
 *  the features will be used for training a classifier
 * 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "mtt/TargetList.h"
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp> 
#include <boost/ref.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/weighted_sum.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <numeric>

using namespace boost::accumulators;

ros::Time time_last_msg(0);
ros::Duration time_elapsed;

tf::TransformListener *p_listener;
tf::StampedTransform transform;

geometry_msgs::Twist twist;
geometry_msgs::Twist features;

//statics
boost::circular_buffer<double> robot_posex_buffer(30);

double robot_x, robot_y;
double robot_vel, robot_theta;

double target_x, target_y;
double target_theta, target_vel;
double distance_to_robot, lateral_disp, sagittal_disp;
double relative_heading, angle_to_robot;
double relative_vel_x, relative_vel_y;

uint leader_tag = 0;
uint target_id = 0;

void targetsCallback(const mtt::TargetList& list)
{  
  static ros::Time start_time = ros::Time::now();
  time_elapsed = ros::Time::now() - start_time;
  
  /// /// ROBOT PART //////
  //use transformations to extract robot features
  try{
    p_listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
    p_listener->lookupTwist("/map", "/base_link", ros::Time(0), ros::Duration(0.5), twist);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  robot_x = transform.getOrigin().x();
  
  robot_posex_buffer.push_back(robot_x);
  
  robot_y = transform.getOrigin().y();
  robot_theta = tf::getYaw(transform.getRotation());
  robot_vel = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));
   
  //robot output line 
  /// uncomment the following for training!
  printf("%d,%d,%.10f,%.10f,%.10f,%.10f,%.10f,0,0,0,0,0,0,0,0\n",
         -1, leader_tag, time_elapsed.toSec(),
         robot_x, robot_y, robot_vel, robot_theta); 

  
  //testing new features extraction
//   accumulator_set<double, stats<tag::variance> > acc;
//   for_each(robot_posex_buffer.begin(), robot_posex_buffer.end(), boost::bind<void>(boost::ref(acc), _1));
//   printf("%f,%f,%f\n", robot_x, mean(acc), sqrt(variance(acc))); 
  
  /// /// TARGETS PART //////
  //sweeps target list and extract features
  for(uint i = 0; i < list.Targets.size(); i++){
    target_id = list.Targets[i].id;
    target_x = list.Targets[i].pose.position.x;
    target_y = list.Targets[i].pose.position.y;
    target_theta = tf::getYaw(list.Targets[i].pose.orientation);
    
    //velocity of target
    target_vel = sqrt(pow(list.Targets[i].velocity.linear.x,2)+
                      pow(list.Targets[i].velocity.linear.y,2));
    
    //angle from target to robot's ref frame
    angle_to_robot = -robot_theta + atan2(target_y - robot_y, 
                                        target_x - robot_x );
    
    //robot and target difference in heading
    relative_heading = robot_theta - target_theta;
    
    //velocity diff from target to robot ref. frame in X axis
    relative_vel_x = (robot_vel - target_vel)*cos(relative_heading);
    
    //velocity diff from target to robot ref. frame in Y axis
    relative_vel_y = (robot_vel - target_vel)*sin(relative_heading);
    
    //distance between robot and target
    distance_to_robot = sqrt(pow(robot_x - target_x,2)+
                        pow(robot_y - target_y,2));
    
    //distance form robot along X axis (robot ref frame)
    lateral_disp = distance_to_robot * sin(angle_to_robot);
    
    //distance form robot along X axis (robot ref frame)
    sagittal_disp = distance_to_robot * cos(angle_to_robot);
          
    //target output (to be used in adaboost training)
    
    // % output file format: 
    // % 1: id
    // % 2: good/bad tag
    // % 3: time
    // % 4: pos x
    // % 5: pos y
    // % 6: heading
    
    // % 7: velocity
    // % 8: lateral disp
    // % 9: relative heading
    // %10: angle to robot
    // %12: distance to robot
    // %13: relative vel x
    // %14: relative vel y
    // %15: sagittal_disp
    
    /// uncomment the following to generate training file!
    printf("%d,%d,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",
      target_id, leader_tag, time_elapsed.toSec(),
      target_x, target_y, target_theta,
      target_vel,
      lateral_disp,
      relative_heading,
      angle_to_robot,
      distance_to_robot,
      relative_vel_x,
      relative_vel_y,
      sagittal_disp);
    
  }
}

void tagCallback(const std_msgs::Header& tag)
{
  //ROS_INFO("tag received");
  //only works for a transition good/bad leader
  leader_tag = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_target");
  ros::NodeHandle n;
  
  tf::TransformListener listener;
  p_listener=&listener;
 
  ros::Subscriber sub = n.subscribe("/targets", 1000, targetsCallback);
  ros::Subscriber sub_tag = n.subscribe("/timetag", 1000, tagCallback);

  ros::spin();

  return 0;
}
