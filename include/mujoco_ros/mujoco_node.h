#ifndef MUJOCO_NODE_H
#define MUJOCO_NODE_H



// ROS
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"

#include "std_srvs/Empty.h"

// MuJoCo
#include "mujoco.h"
#include "glfw3.h"
// Reflexxes
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>



// MuJoCo/GLFW
mjModel* m;                  // MuJoCo model
mjData* d;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

bool glfw_button_left, glfw_button_middle, glfw_button_right;
double glfw_lastx, glfw_lasty;

void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);



class MujocoNode
{
  public:
    MujocoNode();
    ~MujocoNode();
      
    void loop();
  
  private:
    // ROS callbacks
    void jpos_cb(const std_msgs::Float32MultiArray& msg);
    void gripper_cb(const std_msgs::Bool& msg);
    
    bool reset_mujoco_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    
    // gripper
    bool checkGrip(const int& lgI, const int& rgI, const int& targetI);
    int checkGrips(const int& lgI, const int& rgI, const std::vector<int>& geoms);
    void set_grip_weld_relpose(const int& grip_weldI, const int& targetI);
    
    void reset_mujoco();

    //// variables
      
    // UR5
    int UR5_DOF; double UR5_maxVel, UR5_maxAccel, UR5_maxJerk;
    std::vector<std::vector<double> > UR5_jpos, UR5_jvel;
    int UR5_traj_step, UR5_traj_steps; bool UR5_traj_in;
    
    // gripper    
    double gripper_torque, gripper_driver_min_pos;
    bool gripper_in, gripper_state;
    
    
    
    //// from old initialization, for 'reset'

    ReflexxesAPI* rml_api_;
    RMLPositionInputParameters*  rml_i_;
    RMLPositionOutputParameters* rml_o_;
    RMLPositionFlags rml_flags_;
    
    GLFWwindow* window;
    bool ext_cam;
    double FPS_period;
    
    ros::NodeHandle nh_, ur_nh_;
    ros::Subscriber jpos_sub_,gri_sub_;
    ros::Publisher rgb_pub_;
    ros::ServiceServer reset_srv_;

    int UR5_jI_ofs, gripper_mI_ofs,
      gri_l_jI, gri_r_jI, gri_l_gI, gri_r_gI,
      lfinger_eqI_, rfinger_eqI_,
      gripI_, grip_weldI_;
    std::vector<int> grip_bodies_, grip_geoms_;
    
    std::string ext_cam_name; int ext_camI;
};



#endif
