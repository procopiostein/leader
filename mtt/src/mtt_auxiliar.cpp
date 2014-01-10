/**
\file
\brief Auxiliary functions.
*/

#include <mtt/mtt_auxiliar.h>

void init_config(t_config*config)
{
	config->maxr=50.; 		//in meters
	
	config->in_clip=0.1; 		//in meters
	config->out_clip=config->maxr;
	
	config->in_angle= 0; 		//Radians
	config->out_angle= 2*M_PI;		//Radians
	
	// 	config->cluster_break_distance=10;
	// 	config->cluster_break_distance=100;
		config->cluster_break_distance=10.;
// 	config->cluster_break_distance=1000;
// 	config->cluster_break_distance=1000;
// 	config->cluster_break_distance=0.5;
	
	config->fi=5.0*(M_PI/180.);//clustering parameters
// 	config->fi=90*(M_PI/180.);//clustering parameters
	config->beta=60*(M_PI/180.);
// 		config->C0=200.;
	// 	config->C0=2000.;
	config->C0=0.2;
	
	config->filter_max_variation=0.001;//used in raw data filter, in m
	
// 	config->excluding_dr=2.50;
	config->excluding_dr=250;
	
	config->point_occlusion_distance=500.; //used in point occlusion calculation
	// 	config->point_occlusion_distance=2000; //used in point occlusion calculation
	// 	config->point_occlusion_distance=250; //used in point occlusion calculation
	
	config->max_line_per_object=1000;//used in lines
	config->max_mean_variance=0.1;//used in lines
	// 	config->max_mean_variance=5;//used in lines
	
	config->data_acc_scans=20;
	// 	config->data_acc_scans=50;
	
	config->max_missing_iterations=30;
	// 	config->max_missing_iterations=50;
	
	config->overlap_max=5;
	config->overlap_distance=0.2;
	
	config->display_min_lifetime=10;
	
	// 	config->dt=1./13.;
	config->dt=1./30.;
	// 	config->dt=1./40.;
// 	config->dt=1./50.;
	// 	config->dt=1./10.;
	
	config->estimation_window_size=2; //estimation window size for all the error vectors
	
	config->max_stationary_velocity=0.030;
	config->min_moving_velocity=0.100;
	
	config->velocity_acc_size=10;
	
	config->path_lenght=1000;
	
// 	config->default_ellipse_axis=0.500;
// 	config->max_ellipse_axis=2.000;
// 	config->min_ellipse_axis=0.050;
	
	config->default_ellipse_axis=0.500;
	config->max_ellipse_axis=1.500;
	config->min_ellipse_axis=0.050;
	
	config->ezA=1.0;
	config->ezB=0.5;
}

void init_flags(t_flag*flags)
{
	flags->fp_s=true;
	flags->fi=true;	
}

double get_fps(double dt,t_fps*acc)
{
	static bool initialise=true;
	static unsigned int max_size=30;
	
	if(initialise)
	{
		memset(acc->fps,0,max_size*sizeof(double));
		acc->position=0;
		acc->current_size=0;
		
		initialise=false;
	}
	
	acc->fps[acc->position]=1./(double)dt;
	acc->position++;
	
	if(acc->current_size<max_size)
		acc->current_size++;
	
	if(acc->position==max_size)
		acc->position=0;
	
	double mean_fps=0;
	
	for(unsigned int i=0;i<acc->current_size;i++)
		mean_fps+=acc->fps[i];
	
	mean_fps/=acc->current_size;
	
	return mean_fps;
}

int my_tictoc(int status)
{
	static struct timeval timestart;
	struct timeval timenow;
	
	if(status==TIC)
	{
		gettimeofday(&timestart,NULL);
		return 0;
	}else if (status==TOC)
	{
		gettimeofday(&timenow,NULL);
		int ret=timediff(timenow,timestart);
		return ret;
	}else
		return -1;
}

int timediff(struct timeval t1,struct timeval t2)
{
	return (t1.tv_sec-t2.tv_sec)*1000000+(t1.tv_usec-t2.tv_usec);
}

double point2point_distance(double xi,double yi,double xf,double yf)
{	return sqrt((xi-xf)*(xi-xf)+(yi-yf)*(yi-yf));}

double point2point_algebric_distance(double xi,double yi,double xf,double yf)
{	return (xi-xf)*(xi-xf)+(yi-yf)*(yi-yf);}

double point2line_distance(double alpha,double ro,double x,double y)
{	return ro-x*cos(alpha)-y*sin(alpha);}
