#ifndef _MTT_DATA_ASSOCIATION_H_
#define _MTT_DATA_ASSOCIATION_H_
/**
\file
\brief Data association functions header.
*/

#include "mtt_common.h"

#ifdef _MTT_DATA_ASSOCIATION_CPP_
///This variable is a global id to give to new objects, this ensures that new objects have new ids
unsigned int last_id=0; 
#else
extern unsigned int last_id;
#endif

unsigned int GetListSize(t_list*list);

// void AssociateObjects(t_list**list,t_object**objects,int object_size,t_config*config,t_flag*flags);
void AssociateObjects(vector<t_listPtr> &list,vector<t_objectPtr> &objects,t_config& config,t_flag& flags);

void AddObjectToList(vector<t_listPtr> &list,t_object& object,t_config&config);

void RemoveFromList(vector<t_listPtr> &list,unsigned int id);

extern void AllocMotionModels(t_list&list,t_config&config);

void AllocPath(t_path*path,t_config&config);
void InitialiseSearchArea(t_list&list,t_config&config);

void PrintNobjects(t_list*list);

void InitialiseTimers(t_timers*timer);
void InitialiseClassification(t_classification*classification);

/*double CheckAssociationCriteria(t_list*list,t_object*object,t_config*config);
double CheckAssociationCriteria(t_list*list,t_object& object);*/
double CheckAssociationCriteria(t_list&list,t_object& object);

void SingleObjectAssociation(t_list& list,t_object& object);

void AddPointPath(t_path*path,double x,double y);

void SetSearchArea(t_list& list,t_config&config);

void AllocErrors(t_errors*error,t_config&config);

void SetOjectMorphology(t_list&list,t_object& object);

void PrintTree(t_list*list,int l);
extern double point2point_distance(double xi,double yi,double xf,double yf);
extern int real2print(double x,t_config*config);
#endif
