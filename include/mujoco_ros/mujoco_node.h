#ifndef MUJOCO_NODE_H
#define MUJOCO_NODE_H



// ROS
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



#define GLFW_W 640
#define GLFW_H 480
#define GLFW_C 3

#define UR5_DOF 6



// threaded manip
#include "mujoco_ros/ThreadLock.h"

#define JNT_LOCK_TOL 0.005



struct JointIndex
{
  int p, v; // position, velocity
};

struct ThreadedConnection
{
  std::string fastener_name;
  double pitch; JointIndex tI,rI;
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
    // ROS callbacks
    void jpos_cb(const std_msgs::Float32MultiArray& msg);
    void gripper_cb(const std_msgs::Bool& msg);
    
    bool reset_mujoco_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool getpose_cb(mujoco_ros::GetPose::Request& req, mujoco_ros::GetPose::Response& res);
    bool get_brelpose_cb(mujoco_ros::GetRelativePoseBodies::Request& req,
        mujoco_ros::GetRelativePoseBodies::Response& res);
    
    // gripper
    bool checkGrip(const int& target_gI);
    int checkGrips();
    void xpose_to_tf(tf::Transform& tf_out, const int& bI);
    void get_relpose(tf::Transform& tf_out, const int& baI, const int& bbI);
    void set_grip_weld_relpose(const int& target_bI);
    
    // other
    void reset_mujoco();
    
    
    
    // Mujoco/GLFW
    GLFWwindow* window;
    double FPS_period;
    
    // Reflexxes
    ReflexxesAPI* rml_api;
    RMLPositionInputParameters*  rml_i;
    RMLPositionOutputParameters* rml_o;
    RMLPositionFlags rml_flags;
    
    // ROS
    ros::NodeHandle nh, ur_nh;
    ros::Subscriber jpos_sub,gri_sub;
    ros::ServiceServer reset_srv, getpose_srv,get_brelpose_srv;

    // UR5
    double UR5_maxVel, UR5_maxAccel, UR5_maxJerk;
    std::vector<std::vector<double> > UR5_jpos, UR5_jvel;
    int UR5_traj_step, UR5_traj_steps; bool UR5_traj_in;
    std::vector<JointIndex> UR5_jI;
    // gripper    
    double gripper_torque, gripper_driver_min_pos;
    bool gripper_in, gripper_state;
    std::vector<JointIndex> gri_jI;    
    int gripper_m1I,gripper_m2I, gri_l_gI, gri_r_gI,
      lfinger_eqI,rfinger_eqI, grippedI, grip_weldI;
    std::vector<int> grippable_bI, grippable_gI;
    
    // streaming RGB camera
    void fill_rgb_image_msg(sensor_msgs::Image& msg, const unsigned char* buf, const int& buf_sz);
    void publish_cam_rgb(mjrRect& viewport);
  
    ros::Publisher ext_cam_pub;
    bool ext_cam; std::string ext_cam_name;
    int ext_camI;
    


    // threaded manip
    bool threadlock_cb(mujoco_ros::ThreadLock::Request& req, mujoco_ros::ThreadLock::Response& res);
    void handle_threaded_connections();

    ros::ServiceServer threadlock_srv;
    std::vector<ThreadedConnection> threaded_connections;
};



#endif
