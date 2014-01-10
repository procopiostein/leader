#ifndef _MTT_KALMAN_H_
#define _MTT_KALMAN_H_
/**
\file
\brief Kalman estimation related functions header.
*/

#include "mtt_common.h"

extern void SetSearchArea(t_list& list,t_config&config);
extern void AddPointPath(t_path*path,double x,double y);
// void MotionModelsIteration(t_list*list,t_config*config);
void MotionModelsIteration(vector<t_listPtr> &list,t_config& config);

void GetErrorConvariance(t_errors*error);
void AllocMotionModels(t_list&list,t_config&config);
extern int real2print(double x,t_config*config);
extern double point2line_distance(double alpha,double ro,double x,double y);

void UpdateTransitionMatrixCTRV(CvKalman*model,double /*x*/,double /*y*/,double t,double v,double w,double dt);
void UpdateTransitionMatrixCV(CvKalman*model,double dt);
void UpdateTransitionMatrixCV_SC(CvKalman*model,double dt);

void IterateMotionModelCTRV(CvKalman*model,double vm,double wm);
void IterateMotionModelCV(CvKalman*model,double vxm,double vym);
double IterateMotionModelCV_SC(CvKalman*model,double vm);

CvKalman*CreateModelCTRV(void);
CvKalman*CreateModelCV(void);
CvKalman*CreateModelCV_SC(void);

extern double s[6];
extern double vel,theta;
extern int select_object;
extern FILE*fp;
extern double grxy;
void dA_FwdCt(CvKalman*model,double q[6],double dt,double l=2.54);
void dH_FwdCt(CvKalman*model);
void IterateMotionModelFwdCt(CvKalman*model,double z[2]);
CvKalman*CreateModelFwdCt(void);

#endif
