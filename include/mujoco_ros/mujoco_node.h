#ifndef MUJOCO_NODE_H
#define MUJOCO_NODE_H



#include "mujoco_ros/common.h"

// GLFW
#include "glfw3.h"

#define GLFW_W 640
#define GLFW_H 480
#define GLFW_C 3

// ROS
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"

#include "std_srvs/Empty.h"
#include "mujoco_ros/GetPose.h"
#include "mujoco_ros/GetRelativePoseBodies.h"



// UR5 + gripper
#include "mujoco_ros/ur5.h"
#include "mujoco_ros/gripper.h"

// other "modules"
#include "mujoco_ros/freebody_joint_tracking.h"
#include "mujoco_ros/threaded_body_locking.h"



//// randomization

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
    
    void free_mujoco_memory();
    void reset();
    
    // Mujoco/GLFW
    GLFWwindow* window;
    double mujoco_dt;
    

    
    // ROS
    void fill_rgb_image_msg(sensor_msgs::Image& msg, const unsigned char* buf, const int buf_sz);
    void publish_cam_rgb(mjrRect& viewport);
    
    ros::NodeHandle nh, robot_nh;
    ros::ServiceServer reset_srv, getpose_srv,get_brelpose_srv;  
    ros::Publisher ext_cam_pub;
    bool ext_cam; std::string ext_cam_name; int ext_camI;

    // UR5 + gripper
    UR5* ur5; Gripper* gripper;
    
    // other "modules"
    FreeBodyTracker* fb_tracker;
    ThreadedBodyLocker* tb_locker;
    
    
    
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
