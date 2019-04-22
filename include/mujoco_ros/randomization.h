#ifndef RANDOMIZATION_H
#define RANDOMIZATION_H



#include "mujoco_ros/common.h"
#include "mujoco_ros/freebody_joint_tracking.h"

// ROS
#include "mujoco_ros/RandomizeTexturalAttribute.h"
#include "mujoco_ros/RandomizePhysicalAttribute.h"



struct ChildTF
{
  int bI; tf::Transform pose;
};

struct delay_trigger
{
  bool next, now;
};



class Randomizer
{

  public:
    Randomizer(ros::NodeHandle& nh);
    ~Randomizer() {}
    
    bool init();
    void proc(mjModel* m, mjData* d,
      FreeBodyTracker* fb_tracker);
  
  private:

    // ROS
    bool randomize_textural_cb(mujoco_ros::RandomizeTexturalAttribute::Request& req, mujoco_ros::RandomizeTexturalAttribute::Response& res);
    bool randomize_physical_cb(mujoco_ros::RandomizePhysicalAttribute::Request& req, mujoco_ros::RandomizePhysicalAttribute::Response& res);    
    std::vector<bool> handle_request_tex(mjModel* m, mjData* d, const mujoco_ros::RandomizeTexturalAttribute::Request& req);
    std::vector<bool> handle_request_phys(mjModel* m, mjData* d, const mujoco_ros::RandomizePhysicalAttribute::Request& req);

    ros::ServiceServer randtex_srv, randphys_srv;
    std::vector<mujoco_ros::RandomizeTexturalAttribute::Request> randtex_req;
    std::vector<mujoco_ros::RandomizePhysicalAttribute::Request> randphys_req;
    
    // mujoco + free body tracker
    bool childOK(mjModel* m, mjData* d, const int cI, const int pI);
    void restore_child_poses(mjModel* m, mjData* d, FreeBodyTracker* fb_tracker);
    
    std::vector<ChildTF> childTF_shift;
    delay_trigger childTF_trigger;
    
};



#endif
