#ifndef RANDOMIZATION_H
#define RANDOMIZATION_H



#include "mujoco_ros/common.h"
#include "mujoco_ros/freebody_joint_tracking.h"

// ROS
#include "mujoco_ros/RandomizeTexturalAttribute.h"
#include "mujoco_ros/RandomizePhysicalAttribute.h"



struct delay_trigger
{
  bool next, now;
};

struct PCtf
{
  tf::Transform world_p, world_c;
  int p_bI, jpI[6],jvI[6];
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
    std::vector<bool> handle_request_phys(mjModel* m, mjData* d,
      FreeBodyTracker* fb_tracker, const mujoco_ros::RandomizePhysicalAttribute::Request& req);

    ros::ServiceServer randtex_srv, randphys_srv;
    std::vector<mujoco_ros::RandomizeTexturalAttribute::Request> randtex_req;
    std::vector<mujoco_ros::RandomizePhysicalAttribute::Request> randphys_req;
    
    // mujoco + free body tracker
    bool childOK(mjModel* m, mjData* d, const int cI, const int pI);
    void handle_phys_next_step(mjModel* m, mjData* d, FreeBodyTracker* fb_tracker);
    void handle_phys_c_next_step(FreeBodyTracker* fb_tracker);
    void fix_child_pose(mjModel* m, mjData* d, PCtf& pc);
    void fill_fbtrack_flags(FreeBodyTracker* fb_tracker, std::vector<int>& bI, const bool flag);
    
    std::vector<PCtf> phys_child_pose_fix;
    std::vector<int> parent_fbI, child_fbI;
    delay_trigger parent_fbI_trigger, child_fbI_trigger;
    
};

#define GLFW_W 640
#define GLFW_H 480
#define GLFW_C 3



#endif
