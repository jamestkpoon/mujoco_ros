#ifndef MODULE_GRIPPER_H
#define MODULE_GRIPPER_H



#include "mujoco_ros/common.h"
#include "mujoco_ros/freebody_joint_tracking.h"

// ROS
#include "std_msgs/Bool.h"



struct GripperFinger
{
  int joint_posI, motorI, ftip_geomI, lock_weldI;
  double defpose[7];
};



class Gripper
{

  public:
  
    Gripper(ros::NodeHandle& nh);
    ~Gripper() {}
  
    bool init(mjModel* m, mjData* d, ros::NodeHandle& nh,
      const std::vector<GripperFinger>& gf,
      const int grasp_eq_idx, const std::vector<std::string>& graspable_body_names);
    void proc(mjModel* m, mjData* d, ros::NodeHandle& nh,
      FreeBodyTracker* fb_tracker);   

  private:
  
    // ROS
    void grip_cb(const std_msgs::Bool& msg);
    
    ros::Subscriber grip_sub;
    bool grip_cmd;
        
    // mujoco
    void lock_default_pose(mjModel* m, mjData* d, const int fI);
    void set_weld_relpose(mjModel* m, mjData* d, const int weldI);
    void set_weld_relpose(mjModel* m, mjData* d, const int weldI, tf::Transform& tf_out);
    
    int N_FINGERS;
    double driver_torque, driver_min_pos;
    bool gripper_state;
    std::vector<GripperFinger> fingers;
    
    // grasping
    bool grasp_checks(mjModel* m, mjData* d);
    bool grasp_check(mjModel* m, mjData* d, const int target_gI);
    
    int grasp_weldI, grasped_gI;
    std::vector<int> graspable_gI;
    tf::Transform graspedTF; bool fb_track;
    
};



#endif
