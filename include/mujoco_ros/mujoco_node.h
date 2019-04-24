#ifndef MUJOCO_NODE_H
#define MUJOCO_NODE_H



#include "mujoco_ros/common.h"

// GLFW
#include "glfw3.h"

#define GLFW_W 640
#define GLFW_H 480
#define GLFW_C 3

// ROS
#include "std_srvs/Empty.h"
#include "mujoco_ros/GetPose.h"
#include "mujoco_ros/GetRelativePoseBodies.h"

// ros cam
#include "sensor_msgs/Image.h"

// UR5 + gripper
#include "mujoco_ros/ur5.h"
#include "mujoco_ros/gripper.h"

// other "modules"
#include "mujoco_ros/freebody_joint_tracking.h"
#include "mujoco_ros/threaded_body_locking.h"
#include "mujoco_ros/randomization.h"



//// global MuJoCo/GLFW stuff

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



//// node class

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
    
    void free_mujoco_memory();
    void reset();
    
    // Mujoco/GLFW
    GLFWwindow* window;
    double mujoco_dt;
    
    // ROS    
    ros::NodeHandle nh, robot_nh;
    ros::ServiceServer reset_srv, getpose_srv,get_brelpose_srv;
    
    // ROS cam
    void fill_rgb_image_msg(sensor_msgs::Image& msg, const unsigned char* buf, const int buf_sz);
    void publish_cam_rgb(mjrRect& viewport);
    
    ros::Publisher ros_cam_pub;
    bool ros_cam; std::string ros_cam_name; int ros_camI;

    // UR5 + gripper
    UR5* ur5; Gripper* gripper;
    
    // other "modules"
    FreeBodyTracker* fb_tracker;
    ThreadedBodyLocker* tb_locker;
    Randomizer* randomizer;

};



#endif
