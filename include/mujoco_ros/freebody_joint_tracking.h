#ifndef FREEBODY_JOINT_TRACKING_H
#define FREEBODY_JOINT_TRACKING_H



#include "mujoco_ros/common.h"



struct TrackingData
{
  int bI; std::vector<double> pos_tup,pos_tup_l, vel_tup;
};


struct FreeBody
{
  int bI, pbI; JointIndex jI[6];
  TrackingData parent_tracker;
};



class FreeBodyTracker
{
  
  public:
  
    FreeBodyTracker(const double sim_period);
    ~FreeBodyTracker() {}
    
    bool init(mjModel* m, mjData* d,
      const std::vector<std::string>& freebody_names);
    void proc(mjModel* m, mjData* d);
    
    bool set_tracking_parent(const int bI, const int tpbI);
    bool get_tracking_data(const int bI,
      std::vector<double>& pos_tup, std::vector<double>& vel_tup);
    bool shift(mjModel* m, mjData* d, const int bI, const tf::Transform& w_t_tf);
  
  private:

    int find_bI(const int bI);
  
    double dt;
    std::vector<FreeBody> free_bodies; int N_FB;
    
};



double wrap_pi_diff(const double x);



#endif
