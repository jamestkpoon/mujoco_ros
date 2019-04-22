#ifndef FREEBODY_JOINT_TRACKING_H
#define FREEBODY_JOINT_TRACKING_H



#include "mujoco_ros/common.h"



struct FreeBody
{
  int bI, pbI; JointIndex jI[6];
  tf::Transform defpose; std::vector<double> last_pc_tup;
  std::vector<bool> track;
};



class FreeBodyTracker
{
  
  public:
  
    FreeBodyTracker(const double sim_period);
    ~FreeBodyTracker() {}
    
    bool init(mjModel* m, mjData* d,
      const std::vector<std::string>& freebody_names);
    void proc(mjModel* m, mjData* d);
    
    bool set_track_flags(const int bI, const std::vector<bool>& track);
  
  private:
  
    double dt;
    std::vector<FreeBody> free_bodies; int N_FB;
    
};


#endif
