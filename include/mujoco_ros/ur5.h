#ifndef UR5_H
#define UR5_H



#include "mujoco_ros/common.h"

// ROS
#include "std_msgs/Float32MultiArray.h"

// Reflexxes
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>



class UR5
{

  public:
  
    UR5(ros::NodeHandle& nh,
      const double reflexxes_dt);
    ~UR5();
  
    bool init(mjModel* m, mjData* d, ros::NodeHandle& nh,
      const std::vector<std::string>& joint_names);
    void proc(mjModel* m, mjData* d, ros::NodeHandle& nh);
    
  private:
  
    // ROS    
    void jpos_cb(const std_msgs::Float32MultiArray& msg);

    ros::Subscriber jpos_sub;
    std::vector<std::vector<double> > jpos, jvel;
    int traj_step, traj_steps; bool traj_in, traj_started;
    
    ros::Publisher jstate_pub;
    
    // mujoco
    int DOF; std::vector<JointIndex> jI;

    // Reflexxes    
    ReflexxesAPI* rml_api;
    RMLPositionInputParameters*  rml_i;
    RMLPositionOutputParameters* rml_o;
    RMLPositionFlags rml_flags;

    double maxVel, maxAccel, maxJerk;
    
};



#endif
