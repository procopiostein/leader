/**
\file
\brief Header with common structures and includes
*/
#ifndef _MTT_COMMON_H_
#define _MTT_COMMON_H_

//System Includes

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cxcore.hpp>
#include <signal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/lexical_cast.hpp>
#include <cstdlib>
#include <fstream>
#include <map>
#include <cmath>
#include <mtt/colormap.h>

#define _MAX_CLUSTERS_ 5000
#define _MAX_TRACK_LENGHT_ 1000

#define deg2rad(a) (a*M_PI/180.)
#define rad2deg(a) (a*180./M_PI)

#define pi M_PI

enum enum_velocity_classification {MOVING=5, STATIONARY=6};
enum enum_motion_model {CV,CA,MIX};

using namespace ros;
using namespace std;

/**
* @brief This structure all cluster information
*/
struct st_cluster
{
	///id of the cluster
	int id;			
	///start point
	int stp;
	///end point
	int enp;	
	///total number of points
	int n_points;	
	double rmin,tm;
	double cx,cy,cxp,cyp,cxe,cye;
	double r;
	bool partialy_occluded;
	double lenght;
	
	friend ostream& operator<< (ostream &o, const st_cluster &i)
	{
		return o
		<<"id "<<i.id<<endl
		<<"stp "<<i.stp<<" enp "<<i.enp<<endl
		<<"n_points "<<i.n_points<<endl
		<<"rmin "<<i.rmin<<" tm "<<i.tm<<endl
		<<"C(x,y) "<<i.cx<<" "<<i.cy<<endl
		<<"Cp(x,y) "<<i.cxp<<" "<<i.cyp<<endl
		<<"Ce(x,y) "<<i.cxe<<" "<<i.cye<<endl
		<<"r "<<i.r<<endl
		<<"partialy_occluded "<<i.partialy_occluded<<endl
		<<"lenght "<<i.lenght<<endl;
	}
};

typedef st_cluster t_cluster;
typedef boost::shared_ptr<t_cluster> t_clustersPtr;

/**
* @brief This structure has all points coordinates
*/
typedef struct
{
	double x[2160],y[2160];
	double r[2160],t[2160];
	bool flag[2160],flag2[2160];
	bool occlusion_data;
	int initial_position[2160];
	int n_points;
}t_data;

typedef struct
{
	double r;
	double t;
}t_point;

/**
* @brief This structure contains global configurations parameters
*/
typedef struct
{
	double maxr,in_clip,out_clip;
	double in_angle,out_angle;
	
	///Image resolution
	int w,h;							
	
	///Parameters for the dietmayer clustering
	double fi,beta,C0;					
	
	///Threshold for the temporal point match
	double pm_threshold; 				
	
	int max_line_per_object;
	///for the iepf
	double max_mean_variance;
	
	///Number of scans to accumulate
	int data_acc_scans;
	///This is used to avoid calculating the mean in the raw data filter when points are to far apart
	double filter_max_variation;
	///Maximum dr that can exist between two consecutive points in any given object, this value is used to clip points in the accumulator
	double excluding_dr;
	///Maximum number of iterations that an object can failure detection
	int max_missing_iterations;
	///Maximum number of iterations that an object can overlap another
	int overlap_max;
	///Maximum distance between two object for the to overlap each other
	double overlap_distance;
	///Minimum lifetime before the object appears (in iterations)
	unsigned int display_min_lifetime;
	///This variable is used when calculating the occluded points, in mm
	double point_occlusion_distance;
	///This is used to set the size of the error vectors, innovations and residual
	unsigned int estimation_window_size;
	///Time interval between two consecutive iterations
	double dt;
	
	unsigned int velocity_acc_size;
	
	double max_stationary_velocity;
	double min_moving_velocity;
	
	unsigned int path_lenght;
	double cluster_break_distance;
	
	double default_ellipse_axis;
	double max_ellipse_axis;
	double min_ellipse_axis;
	
	double ezA;
	double ezB;
	
}t_config;

/**
* @brief This structure contains a single line properties
*/
struct st_line
{
	///Polar coordinates
	double ro,alpha;
	///Initial and final line points
	double xi,yi,xf,yf;
};

typedef st_line t_line;
typedef boost::shared_ptr<t_line> t_linePtr;

/**
* @brief This structure contains object information
*/
struct st_object
{
	vector<t_linePtr> lines;
	double cx,cy;
	
	double rmin,tm;
	double size;
	
	int id;
	
	bool object_found;
	bool partialy_occluded;
	
	double min_distance_to_existing_object;
	
	friend ostream& operator<< (ostream &o, const st_object &i)
	{
		return o
		<<"C(x,y) "<<i.cx<<" "<<i.cy<<endl
		<<"rmin "<<i.rmin<<" tm "<<i.tm<<endl
		<<"size "<<i.size<<endl
		<<"n_lines "<<i.lines.size()<<endl
		<<"id "<<i.id<<endl
		<<"object_found "<<i.object_found<<endl
		<<"partialy_occluded "<<i.partialy_occluded<<endl
		<<"min_distance_to_existing_object "<<i.min_distance_to_existing_object<<endl;
	}
};

typedef st_object t_object;
typedef boost::shared_ptr<t_object> t_objectPtr;


typedef struct
{
	unsigned int lifetime;
	int occludedtime;
}t_timers;


typedef struct
{
	CvKalman*cv_x_kalman;
	CvKalman*cv_y_kalman;
	
	CvKalman*ca_x_kalman;
	CvKalman*ca_y_kalman;
}t_motion_models;

typedef struct
{
	double*x_innovation;
	double*x_residue;
	
	double*y_innovation;
	double*y_residue;
	
	double*lateral_error;
	
	double x_inno_cov;
	double x_resi_cov;
	
	double y_inno_cov;
	double y_resi_cov;
	
	double lateral_error_cov;
	
	unsigned int number_points;
	unsigned int max_number_points;
	unsigned int position;
	unsigned int latest;
	unsigned int next;
}t_errors;

typedef struct
{
	double*x;
	double*y;
	unsigned int number_points;
	unsigned int max_number_points;
	unsigned int position;
	unsigned int latest;
	unsigned int next;
}t_path;

typedef struct
{
	double velocity_x;
	double velocity_y;
	double velocity_module;
	double velocity_angle;
}t_velocity;

typedef struct
{
	double ellipse_A;
	double ellipse_B;
	double angle;
	double center_x,center_y;
}t_search_area;

typedef struct
{
	double apparent_size;
}t_object_morphology;

typedef struct
{
	enum_velocity_classification velocity_classification;
	bool occluded;
	bool partialy_occluded;
}t_classification;

/**
* @brief This structure contains global flags parameters
*/
typedef struct
{
	///first point of scan, first image
	bool fp_s,fi,raw_only;	
}t_flag;

typedef struct
{
	double x;
	double y;
}t_measurements;

typedef struct
{
	double estimated_x;
	double estimated_y;
	double predicted_x;
	double predicted_y;
}t_position;


/**
* @brief Full description of and tracked object
*/
struct t_linked_list
{
	unsigned int id;
	t_search_area search_area;
	t_path path_cv;
	t_path path_ca;
	t_velocity velocity;
	t_motion_models motion_models;
	t_timers timers;
	t_errors errors_cv;
	t_errors errors_ca;
	t_object_morphology object_morphology;
	t_classification classification;
	t_measurements measurements;
	t_position position;
	t_object shape;
	enum_motion_model model;
};

typedef t_linked_list t_list;
typedef boost::shared_ptr<t_list> t_listPtr;

typedef struct
{
	double fps[100];
	unsigned int position;
	unsigned int current_size;
}t_fps;

#endif