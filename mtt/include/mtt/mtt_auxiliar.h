#ifndef _MTT_AUXILIAR_H_
#define _MTT_AUXILIAR_H_
/**
\file
\brief Auxiliary functions header
*/

#define mtic my_tictoc(1)
#define mtoc my_tictoc(0)
#define TIC 1
#define TOC 0

#include "mtt_common.h"

double get_fps(double dt,t_fps*acc);

/**
@brief Init flags

@param flags general flags structure
@return void
*/
void init_flags(t_flag*flags);

/**
@brief Init configuration

@param config general configuration structure
@return void
*/
void init_config(t_config*config);

/**
@brief TIC TOC implementation functions

@param status TIC or TOC
@return timediff in useconds if TOC, 0 if TIC, -1 if unknown
*/
int my_tictoc(int status);

/**
@brief Calculates timediff

@param t1 first time
@param t2 second time
@return timediff in useconds
*/
int timediff(struct timeval t1,struct timeval t2);

/**
@brief Calculates the line to point distance

@param alpha polar coordinates of the line
@param ro polar coordinates of the line
@param x cartesian coordinates of the point
@param y
@return algebric distance
*/
double point2line_distance(double alpha,double ro,double x,double y);

/**
@brief Calculates the algebric distante between two points.

@param xi
@param yi
@param xf
@param yf
@return algebric distance
*/
double point2point_algebric_distance(double xi,double yi,double xf,double yf);

/**
@brief Calculates the distante between two points.

@param xi
@param yi
@param xf
@param yf
@return distance
*/
double point2point_distance(double xi,double yi,double xf,double yf);

void CreateMeasurementFromDisplacement(double dx,double dy,double dtheta,double z[2],double dt,double l,double bwa);
void ConvertEstimatedToMeasurment(double vl,double dir,float*dx,float*dy,float*dtheta,double dt,double l,double bwa);

#endif
