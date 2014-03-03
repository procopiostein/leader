// #include "ros/ros.h"
// #include "tf/tf.h"
// #include "leader_follower.h"

// int main(int argc, char **argv){ 
//   ros::init(argc, argv, "leader_follower");
//   ros::NodeHandle n;
// 
//   //used in transformations
//   tf::TransformListener _listener;
//   listener = &_listener;
//   
//   //initialize leader_path
//   leader_path.header.stamp = ros::Time::now();
//   leader_path.header.frame_id = "/map";
//   
//   //tell the action client that we want to spin a thread by default
// //   MoveBaseClient ac("robot_0/move_base", true);
//   
//   //subscribers
//   dyn_objects_subscriber = n.subscribe("dynamic_objects", 1, &dyn_objects_callback);
//   leader_goal_subscriber = n.subscribe("leader_goal_pose",1,&leader_goal_callback);
//   robot_goal_subscriber = n.subscribe("/robot_0/goal", 1, &robot_goal_callback);
//   robot_pose_subscriber = n.subscribe("/robot_0/base_pose_ground_truth", 1, &robot_pose_callback);
//   
//   //publishers
//   pathPublisher = n.advertise<nav_msgs::Path>("leader_path", 1);
//   robot_cmd_vel = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1);
//   next_posePublisher = n.advertise<geometry_msgs::PoseStamped>("/robot_0/leader_next_pose", 1);
//   
//   //initialize time counter for goal passed
//   t_wait = ros::Time::now().toSec();
//   
//   ros::Rate loop_rate(10);
//   
//   while (ros::ok()){
// 
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }


#include <cstdlib>
#include "opencv/cv.h"
#include "opencv/ml.h"
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

  /* STEP 2. Opening the file */
  //1. Declare a structure to keep the data
  CvMLData cvml;
  //2. Read the file
  cvml.read_csv("samples.csv");
  //3. Indicate which column is the response
  cvml.set_response_idx(0);

  /* STEP 3. Splitting the samples */
  //1. Select 40 for the training
//   CvTrainTestSplit cvtts(7000, true);
  //2. Assign the division to the data
//   cvml.set_train_test_split(&cvtts);

  printf("Training ... ");
  /* STEP 4. The training */
  //1. Declare the classifier
  CvBoost boost;
  //2. Train it with 100 features
  boost.train(&cvml, CvBoostParams(CvBoost::DISCRETE, 500, 0, 1, false, 0), false);

  /* STEP 5. Calculating the testing and training error */
  // 1. Declare a couple of vectors to save the predictions of each sample
  std::vector<float> train_responses, test_responses;
  // 2. Calculate the training error
  float fl1 = boost.calc_error(&cvml,CV_TRAIN_ERROR,&train_responses);
  // 3. Calculate the test error
  float fl2 = boost.calc_error(&cvml,CV_TEST_ERROR,&test_responses);
  printf("Error train %f \n", fl1);
  printf("Error test %f \n", fl2);

  /* STEP 6. Save your classifier */
  // Save the trained classifier
  boost.save("./trained_boost.xml", "boost");

  return EXIT_SUCCESS;
}
