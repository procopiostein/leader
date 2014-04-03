#ifndef PROXEMIC_LAYER_H_
#define PROXEMIC_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <people_msgs/People.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <social_navigation_layers/ProxemicLayerConfig.h>

namespace social_navigation_layers
{
  class ProxemicLayer : public costmap_2d::Layer
  {
    public:
      ProxemicLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      bool isDiscretized() { return false; }

    private:
      void peopleCallback(const people_msgs::People& people);
      void configure(ProxemicLayerConfig &config, uint32_t level);
      ros::Subscriber people_sub_;
      people_msgs::People people_list_;
      std::list<people_msgs::Person> transformed_people_;
      double cutoff_, amplitude_, covar_, factor_;
      ros::Duration people_keep_time_;
      boost::recursive_mutex lock_;
      tf::TransformListener tf_;
      dynamic_reconfigure::Server<ProxemicLayerConfig>* server_;
      dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_;
  };
};
#endif

