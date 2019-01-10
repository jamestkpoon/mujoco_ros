// ROS
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
// MuJoCo
#include "mujoco.h"
#include "glfw3.h"
// Reflexxes
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

// UR5 joint offsets
const double UR5_joint_offsets[6] = { 0.0, M_PI, 0.0, M_PI, 0.0, 0.0 };

// Reflexxes for UR5 control
const int UR5_DOF = 6;
const double UR5_maxVel =  180 * (M_PI/180),
  UR5_maxAccel = 40 * (M_PI/180), UR5_maxJerk = 80 * (M_PI/180); // from v-rep
std::vector<std::vector<double> > UR5_jpos, UR5_jvel;
int UR5_traj_step, UR5_traj_steps; bool UR5_traj_in = false;
// gripper control
const double gripper_torque = 1.0, gripper_driver_min_pos = 0.05;
bool gripper_in = false, gripper_state = false;

// MuJoCo/GLFW
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
  
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_left )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_middle )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// ROS callbacks

void jpos_cb(const std_msgs::Float32MultiArray& msg)
{
  int nsteps_ = msg.data.size() / UR5_DOF;
  
  double vC_ = 0.0;
  if(msg.data.size()%UR5_DOF != 0) vC_ = msg.data.back();
  
  if(nsteps_ > 0)
  {
    // target joint positions
    UR5_jpos.clear(); int i_ = 0;
    for(int i=0; i<nsteps_; i++)
    {
      std::vector<double> d_(UR5_DOF);
      for(int j=0; j<UR5_DOF; j++) d_[j] = msg.data[i_++] - UR5_joint_offsets[j];
      UR5_jpos.push_back(d_);
    }
    
    // target joint velocities
    UR5_jvel.clear();
    for(int i=0; i<nsteps_-1; i++)
    {
      std::vector<double> d_(UR5_DOF);
      for(int j=0; j<UR5_DOF; j++) d_[j] = vC_ * (UR5_jpos[i+1][j]-UR5_jpos[i][j]);
      UR5_jvel.push_back(d_);
    }
    UR5_jvel.push_back(std::vector<double>(UR5_DOF, 0.0));
    
    UR5_traj_step = 0; UR5_traj_steps = nsteps_; UR5_traj_in = true;
  }  
}

void gripper_cb(const std_msgs::Bool& msg)
{
  gripper_in = msg.data;
}

int main(int argc, char **argv)
{
  //// ROS stuff
  
  ros::init(argc, argv, "mujoco_ur5");
  ros::NodeHandle nh_;
    
  ros::Subscriber jpos_sub_ = nh_.subscribe("/mujoco/ur5/command/joint_positions", 1, jpos_cb),
    gri_sub_ = nh_.subscribe("/mujoco/ur5/command/gripper", 1, gripper_cb);
  
  //// MuJoCo/GLFW stuff
  
  // activate MuJoCo
  std::string mujoco_key_; nh_.getParam("/mujoco_key", mujoco_key_);
  mj_activate(mujoco_key_.c_str());

  // load model
  std::string mujoco_xml_; nh_.getParam("/mujoco_xml", mujoco_xml_);
  char error[1000];
  m = mj_loadXML(mujoco_xml_.c_str(), NULL, error, 1000);
  d = mj_makeData(m);

  // initialize GLFW
  glfwInit();
  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Mujoco_UR5", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // set GLFW callbacks
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
  
  // useful indexing
  int UR5_jI_ofs = mj_name2id(m, mjOBJ_JOINT, "joint1"), // UR5 joints
    gripper_mI_ofs = mj_name2id(m, mjOBJ_ACTUATOR, "close_1"), // gripper motors
    gri_l_jI = mj_name2id(m, mjOBJ_JOINT, "joint7_2"), // gripper drive hinges
    gri_r_jI = mj_name2id(m, mjOBJ_JOINT, "joint7_1");
    
  // "settle" the simulation by starting at a certain timestamp
  double settle_time_; nh_.getParam("/mujoco_start_time", settle_time_);
  while(d->time < settle_time_) mj_step(m, d);
  
  ////
  
  //// Reflexxes stuff
  ReflexxesAPI *rml_api_ = NULL;
  RMLPositionInputParameters  *rml_args_i_ = new RMLPositionInputParameters(UR5_DOF);
  RMLPositionOutputParameters *rml_args_o_ = new RMLPositionOutputParameters(UR5_DOF);
  RMLPositionFlags rml_flags_;
  
  for(int i=0; i<UR5_DOF; i++)
  {
    rml_args_i_->CurrentPositionVector->VecData[i] = d->qpos[i+UR5_jI_ofs];
    rml_args_i_->CurrentVelocityVector->VecData[i] = d->qvel[i+UR5_jI_ofs];
    rml_args_i_->CurrentAccelerationVector->VecData[i] = 0.0;
            
    rml_args_i_->MaxVelocityVector->VecData[i] = UR5_maxVel;
    rml_args_i_->MaxAccelerationVector->VecData[i] = UR5_maxAccel;
    rml_args_i_->MaxJerkVector->VecData[i] = UR5_maxJerk;
    
    rml_args_i_->SelectionVector->VecData[i] = true;
  }
  
  ////

  //// main loop
  
  int fps_; nh_.getParam("/mujoco_FPS", fps_); double FPS_period = 1.0 / fps_;
  
  while(!glfwWindowShouldClose(window) && ros::ok())
  {
    // ROS
    ros::spinOnce();
    
    // UR5 joint control
    if(UR5_traj_in)
    {
      if(rml_api_ == NULL) // start new motion
      {
        rml_api_ = new ReflexxesAPI(UR5_DOF, FPS_period);
        nh_.setParam("/mujoco/ur5/moving", true);
        // set target position and velocity
        for(int i=0; i<UR5_DOF; i++)
        {          
          rml_args_i_->TargetPositionVector->VecData[i] = UR5_jpos[UR5_traj_step][i];
          rml_args_i_->TargetVelocityVector->VecData[i] = UR5_jvel[UR5_traj_step][i];
        }
      }
      else // continue motion
      {
        // update
        int rml_result_ = rml_api_->RMLPosition(*rml_args_i_,rml_args_o_, rml_flags_);
        *rml_args_i_->CurrentPositionVector = *rml_args_o_->NewPositionVector;
        *rml_args_i_->CurrentVelocityVector = *rml_args_o_->NewVelocityVector;
        *rml_args_i_->CurrentAccelerationVector = *rml_args_o_->NewAccelerationVector;
        // check if finished
        if(rml_result_ == ReflexxesAPI::RML_FINAL_STATE_REACHED)
          { free(rml_api_); rml_api_ = NULL; UR5_traj_step++; }

        // copy UR5 state to MuJoCo
        for(int i=0; i<UR5_DOF; i++)
        {
          d->qpos[i+UR5_jI_ofs] = rml_args_o_->NewPositionVector->VecData[i];
          d->qvel[i+UR5_jI_ofs] = rml_args_o_->NewVelocityVector->VecData[i];
        }
      }

      if(UR5_traj_step == UR5_traj_steps)
        { nh_.setParam("/mujoco/ur5/moving", false); UR5_traj_in = false; }
    }
    
    // gripper control
    if(gripper_state != gripper_in)
    {
      if(gripper_in) // close gripper
        d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = gripper_torque;
      else
        d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = -gripper_torque;
        
      gripper_state = gripper_in;
    }
    
    if(!gripper_state && std::min(d->qpos[gri_l_jI], d->qpos[gri_r_jI]) < gripper_driver_min_pos)
      d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = 0.0;
      
    // update MuJoCo
    mjtNum simstart = d->time;
    while( d->time - simstart < FPS_period ) mj_step(m, d);
    
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }
  
  //// MuJoCo cleanup

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deactivate();

  // close window, stop GLFW
  if(!glfwWindowShouldClose(window)) glfwSetWindowShouldClose(window, GLFW_TRUE);
  glfwTerminate();
  
  ////
  
  //// Reflexxes cleanup
  
  delete rml_api_; delete rml_args_i_; delete rml_args_o_;
  
  ////
  
  return 0;
}
