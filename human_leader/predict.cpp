#include <cstdlib>
#include "opencv/cv.h"
#include "opencv/ml.h"
#include <vector>
// #include <stdio>

using namespace std;
using namespace cv;

#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

int main (int argc, char * const argv[])
{   
  Mat input;
  Mat sample;/* = cvMat( 1, 6, CV_32F );*/

  string demoFile  = "test.yaml";

  FileStorage fsDemo( demoFile, FileStorage::READ);
  fsDemo["temp"] >> input;

//   cout << "Print the contents of input:" << endl;
//   cout << input << endl << endl;

  CvBoost boost;
  boost.load("trained_boost.xml");

  int length = input.rows;
//   cout << "length:" << length << endl;
  
  for(int i=0; i<length; i++){
//     cout << "copy row:" << i << endl;
    sample.push_back(input.row(i));
    
//     cout << "contents of sample:" << endl;
//     cout << sample << endl << endl;

    double x = boost.predict(sample);
//     cout << "label:" << x << endl;
    cout << x << endl;
    
    sample.pop_back(1);
  }
  
  fsDemo.release();
  return 0;
}

// int main(int argc, char** argv) {
// 
//   cout << "sample: " 
//     << argv[1] << " "
//     << argv[2] << " " 
//     << argv[3] << endl;
//   

//   
//   
//   
//   CvMat* temp_sample = 0;
//   CvMat* weak_responses = 0;
//   temp_sample = cvCreateMat( 1, 22, CV_32F );
//   weak_responses = cvCreateMat( 1, boost.get_weak_predictors()->total, CV_32F );
// 
//   temp_sample->data.fl[1] = atof(argv[1]);
//   temp_sample->data.fl[2] = atof(argv[2]);
//   temp_sample->data.fl[3] = atof(argv[4]);
//   temp_sample->data.fl[1] = atof(argv[5]);
//   temp_sample->data.fl[2] = atof(argv[6]);
//   temp_sample->data.fl[3] = atof(argv[7]);
//   temp_sample->data.fl[1] = atof(argv[8]);
//   temp_sample->data.fl[2] = atof(argv[9]);
//   temp_sample->data.fl[3] = atof(argv[10]);
//   temp_sample->data.fl[1] = atof(argv[11]);
//   temp_sample->data.fl[2] = atof(argv[12]);
//   temp_sample->data.fl[3] = atof(argv[14]);
//   temp_sample->data.fl[1] = atof(argv[15]);
//   temp_sample->data.fl[2] = atof(argv[16]);
//   temp_sample->data.fl[3] = atof(argv[17]);
//   temp_sample->data.fl[1] = atof(argv[18]);
//   temp_sample->data.fl[2] = atof(argv[19]);
//   temp_sample->data.fl[3] = atof(argv[20]);
//   temp_sample->data.fl[3] = atof(argv[21]);
//   
// //   Mat sample = Mat(1,4, CV_32F );
// //   sample.at<float>(0,1)=0;
// //   sample.at<float>(0,2)=255;
// //   sample.at<float>(0,3)=255;
// //   sample.at<float>(0,4)=255;
// 
//   cout<<"will predict"<<endl;
// //   float x = boost.predict(sample, Mat(), Range::all(), false, false);
//   double x = boost.predict( temp_sample, 0, weak_responses );
//   double sum = cvSum( weak_responses ).val[0];
//   
//   cout << "class:" << x << " sum:" << sum << endl;
// 
// }