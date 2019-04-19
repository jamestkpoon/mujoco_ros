#ifndef FREEBODY_JOINT_TRACKING_H
#define FREEBODY_JOINT_TRACKING_H



#include "mujoco_ros/common.h"



struct FreeBody
{
  int bI, pbI; tf::Transform defpose;
  JointIndex jI[6]; std::vector<bool> track;
};



class FreeBodyTracker
{
  
  public:
  
    FreeBodyTracker() {}
    ~FreeBodyTracker() {}
    
    bool init(mjModel* m, mjData* d,
      const std::vector<std::string>& freebody_names);
    void proc(mjModel* m, mjData* d);
    
    bool set_track_flags(const int bI, const std::vector<bool>& track);
  
  private:
  
    std::vector<FreeBody> free_bodies;
    int N_FB;
};


#endif
