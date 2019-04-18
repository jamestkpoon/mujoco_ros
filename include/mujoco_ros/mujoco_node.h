#ifndef MUJOCO_NODE_H
#define MUJOCO_NODE_H



// ROS comms
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"

#include "std_srvs/Empty.h"
#include "mujoco_ros/GetPose.h"
#include "mujoco_ros/GetRelativePoseBodies.h"

// MuJoCo
#include "mujoco.h"
#include "glfw3.h"
// Reflexxes
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


// defines
#define GLFW_W 640
#define GLFW_H 480
#define GLFW_C 3

#define UR5_DOF 6



// free bodies
struct JointIndex
{
  int I, p,v; // id,  position/velocity indices
};

struct FreeBody
{
  int bI; tf::Transform tf_pose, tf_defpose;
  JointIndex jI[6]; bool proc[6];
};



// threaded manip
#include "mujoco_ros/ThreadLock.h"

#define JNT_LOCK_TOL 0.005

struct ThreadedConnection
{
  std::string fastener_name;
  double pitch; JointIndex jI[6];
};



// randomization
#include "mujoco_ros/RandomizeTexturalAttribute.h"
#include "mujoco_ros/RandomizePhysicalAttribute.h"

double rand_01()
{
  // https://stackoverflow.com/a/6219525
  return (double)rand() / (double)((unsigned)RAND_MAX);
}

double rand_pm1()
{
  return 2*rand_01() - 1.0;
}

struct PCtf
{
  tf::Transform world_p, world_c;
  int p_bI, jpI[6],jvI[6];
};



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
    // misc
    bool reset_mujoco_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool getpose_cb(mujoco_ros::GetPose::Request& req, mujoco_ros::GetPose::Response& res);
    bool get_brelpose_cb(mujoco_ros::GetRelativePoseBodies::Request& req, mujoco_ros::GetRelativePoseBodies::Response& res);
    
    void free_memory();
    void get_indexing();
    void reset_mujoco(bool init);

    void xpose_to_tf(tf::Transform& tf_out, const int bI);
    void get_relpose(tf::Transform& tf_out, const int abI, const int bbI);
    
    // Mujoco/GLFW
    GLFWwindow* window;
    double FPS_period;
    
    // Reflexxes
    ReflexxesAPI* rml_api;
    RMLPositionInputParameters*  rml_i;
    RMLPositionOutputParameters* rml_o;
    RMLPositionFlags rml_flags;
    
    // ROS comms
    ros::NodeHandle nh, ur_nh;
    ros::Subscriber jpos_sub,gri_sub;
    ros::ServiceServer reset_srv, getpose_srv,get_brelpose_srv;

    // UR5
    void jpos_cb(const std_msgs::Float32MultiArray& msg);
    void handle_UR5();
    
    double UR5_maxVel, UR5_maxAccel, UR5_maxJerk;
    std::vector<std::vector<double> > UR5_jpos, UR5_jvel;
    int UR5_traj_step, UR5_traj_steps; bool UR5_traj_in, UR5_traj_started;
    std::vector<JointIndex> UR5_jI;
    ros::Publisher jstate_pub;
    
    // gripper
    void gripper_cb(const std_msgs::Bool& msg);
    void handle_gripper();
    bool gripper_check(const int target_gI);
    int gripper_checks();
    void gripper_set_weld_relpose(const int weldI);
    void gripper_lock_default();
        
    double gripper_torque, gripper_driver_min_pos;
    bool gripper_in, gripper_state;
    std::vector<JointIndex> gri_jI;    
    int gripper_m1I, gripper_m2I, // motor indices
      gri_l_gI, gri_r_gI, // fingertip geom indices for grasp contact check
      gri_l_weldI,gri_r_weldI, grip_weldI, grippedI; // weld equality indices
    double gri_l_defpose[7], gri_r_defpose[7]; // default fingertip poses
    std::vector<std::string> grippable_body_names;
    std::vector<int> grippable_bI, grippable_gI;
    
    // streaming RGB camera
    void fill_rgb_image_msg(sensor_msgs::Image& msg, const unsigned char* buf, const int buf_sz);
    void publish_cam_rgb(mjrRect& viewport);
  
    ros::Publisher ext_cam_pub;
    bool ext_cam; std::string ext_cam_name;
    int ext_camI;
    
    
    
    // free bodies
    void handle_free_bodies();
    std::vector<FreeBody> free_bodies;
    


    // threaded manip
    bool threadlock_cb(mujoco_ros::ThreadLock::Request& req, mujoco_ros::ThreadLock::Response& res);
    void handle_threaded_connections();

    ros::ServiceServer threadlock_srv;
    std::vector<ThreadedConnection> threaded_connections;
    
    
    
    // randomization
    bool randomize_textural_cb(mujoco_ros::RandomizeTexturalAttribute::Request& req, mujoco_ros::RandomizeTexturalAttribute::Response& res);
    bool randomize_physical_cb(mujoco_ros::RandomizePhysicalAttribute::Request& req, mujoco_ros::RandomizePhysicalAttribute::Response& res);
    double rand_clip(const double mean, const double noise, const double lb, const double ub);
    bool rand_childOK(const int cI, const int pI);
    void handle_randomization();
    void rand_child_pose_fix(PCtf& pc);
    
    ros::ServiceServer randtex_srv, randphys_srv;
    std::vector<PCtf> rand_child_fix; bool rand_proc_now;
};



#endif
