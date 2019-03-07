// ROS
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
// MuJoCo
#include "mujoco.h"
#include "glfw3.h"
// Reflexxes
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

//// Reflexxes for UR5 control

const int UR5_DOF = 6;
const double UR5_maxVel =  180 * (M_PI/180),
  UR5_maxAccel = 40 * (M_PI/180), UR5_maxJerk = 80 * (M_PI/180); // from v-rep
std::vector<std::vector<double> > UR5_jpos, UR5_jvel;
int UR5_traj_step, UR5_traj_steps; bool UR5_traj_in = false;
// gripper control
const double gripper_torque = 1.0, gripper_driver_min_pos = 0.05;
bool gripper_in = false, gripper_state = false;

////

//// MuJoCo/GLFW

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

////

//// ROS callbacks

void jpos_cb(const std_msgs::Float32MultiArray& msg)
{
  int nsteps_ = msg.data.size() / UR5_DOF;
  
  if(nsteps_ > 0)
  {
    // target joint positions
    UR5_jpos.clear(); int i_ = 0;
    for(int i=0; i<nsteps_; i++)
    {
      std::vector<double> d_(UR5_DOF);
      for(int j=0; j<UR5_DOF; j++) d_[j] = msg.data[i_++];
      UR5_jpos.push_back(d_);
    }
    
    // target joint velocities
    double vC_ = 1.0;
    if(msg.data.size()%UR5_DOF == 1) vC_ = msg.data.back();
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

////

//// helper fns

bool checkGrip(const int& lgI, const int& rgI, const int& targetI)
{
  bool lcon_ = false, rcon_ = false;
  for(int i=0; i<d->ncon; i++)
  {
    if(!lcon_ && (d->contact[i].geom1 == lgI || d->contact[i].geom2 == lgI))
      lcon_ = (d->contact[i].geom1 == targetI || d->contact[i].geom2 == targetI);
    if(!rcon_ && (d->contact[i].geom1 == rgI || d->contact[i].geom2 == rgI))
      rcon_ = (d->contact[i].geom1 == targetI || d->contact[i].geom2 == targetI);
  }
  
  return (lcon_ && rcon_);
}

int checkGrips(const int& lgI, const int& rgI, const std::vector<int>& geoms)
{
  for(int i=0; i<(int)geoms.size(); i++)
    if(checkGrip(lgI,rgI, geoms[i])) return i;
    
  return -1;
}

void set_grip_weld_relpose(const int& grip_weldI, const int& targetI)
{
  int eeI_ = m->eq_obj1id[grip_weldI]; tf::Transform ee_tf_, target_tf_;
  ee_tf_.setOrigin(tf::Vector3(d->xpos[eeI_*3+0],d->xpos[eeI_*3+1],d->xpos[eeI_*3+2]));
  ee_tf_.setRotation(tf::Quaternion(d->xquat[eeI_*4+1],d->xquat[eeI_*4+2],d->xquat[eeI_*4+3],d->xquat[eeI_*4+0]));
  target_tf_.setOrigin(tf::Vector3(d->xpos[targetI*3+0],d->xpos[targetI*3+1],d->xpos[targetI*3+2]));
  target_tf_.setRotation(tf::Quaternion(d->xquat[targetI*4+1],d->xquat[targetI*4+2],d->xquat[targetI*4+3],d->xquat[targetI*4+0]));
  
  tf::Transform relpose_ = ee_tf_.inverse() * target_tf_;
  m->eq_data[7*grip_weldI+0] = relpose_.getOrigin()[0];
  m->eq_data[7*grip_weldI+1] = relpose_.getOrigin()[1];
  m->eq_data[7*grip_weldI+2] = relpose_.getOrigin()[2];
  m->eq_data[7*grip_weldI+3] = relpose_.getRotation()[3];
  m->eq_data[7*grip_weldI+4] = relpose_.getRotation()[0];
  m->eq_data[7*grip_weldI+5] = relpose_.getRotation()[1];
  m->eq_data[7*grip_weldI+6] = relpose_.getRotation()[2];
}

////

int main(int argc, char **argv)
{
  //// ROS stuff
  
  ros::init(argc, argv, "mujoco_ur5");
  ros::NodeHandle nh_, ur_nh_("ur5");
    
  ros::Subscriber jpos_sub_ = ur_nh_.subscribe("command/joint_positions", 1, jpos_cb),
    gri_sub_ = ur_nh_.subscribe("command/gripper", 1, gripper_cb);
  
  //// MuJoCo/GLFW stuff
  
  // activate MuJoCo
  std::string mujoco_key_; nh_.getParam("key", mujoco_key_);
  mj_activate(mujoco_key_.c_str());

  // load model
  std::string mujoco_xml_; nh_.getParam("xml", mujoco_xml_);
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
  
  // "settle" the simulation by starting at a certain timestamp
  double settle_time_; nh_.getParam("start_time", settle_time_);
  while(d->time < settle_time_) mj_step(m, d);
  
  // other stuff
  int fps_; nh_.getParam("FPS", fps_); double FPS_period = 1.0 / fps_;
  
  int UR5_jI_ofs = mj_name2id(m, mjOBJ_JOINT, "joint1"), // UR5 joints
    gripper_mI_ofs = mj_name2id(m, mjOBJ_ACTUATOR, "close_1"), // gripper motors
    gri_l_jI = mj_name2id(m, mjOBJ_JOINT, "joint7_2"), // gripper drive hinges
    gri_r_jI = mj_name2id(m, mjOBJ_JOINT, "joint7_1"),
    gri_l_gI = mj_name2id(m, mjOBJ_GEOM, "left_follower_mesh"), // gripper finger geoms
    gri_r_gI = mj_name2id(m, mjOBJ_GEOM, "right_follower_mesh"),
    lfinger_eqI_ = mj_name2id(m, mjOBJ_EQUALITY, "lfinger_lock"), // gripper finger welds
    rfinger_eqI_ = mj_name2id(m, mjOBJ_EQUALITY, "rfinger_lock");
  
  //// Reflexxes stuff
  
  ReflexxesAPI *rml_api_ = NULL;
  RMLPositionInputParameters  *rml_i_ = new RMLPositionInputParameters(UR5_DOF);
  RMLPositionOutputParameters *rml_o_ = new RMLPositionOutputParameters(UR5_DOF);
  RMLPositionFlags rml_flags_;
  
  for(int i=0; i<UR5_DOF; i++)
  {
    rml_i_->CurrentPositionVector->VecData[i] = d->qpos[i+UR5_jI_ofs];
    rml_i_->CurrentVelocityVector->VecData[i] = d->qvel[i+UR5_jI_ofs];
    rml_i_->CurrentAccelerationVector->VecData[i] = 0.0;
            
    rml_i_->MaxVelocityVector->VecData[i] = UR5_maxVel;
    rml_i_->MaxAccelerationVector->VecData[i] = UR5_maxAccel;
    rml_i_->MaxJerkVector->VecData[i] = UR5_maxJerk;
    
    rml_i_->SelectionVector->VecData[i] = true;
  }
  
  ur_nh_.setParam("moving", false);

  //// main loop
  
  // gripper init
  std::vector<std::string> grip_body_names_; nh_.getParam("grabbable_bodies", grip_body_names_);
  int n_grip_bodies_ = (int)grip_body_names_.size(),
    grip_weldI_ = mj_name2id(m, mjOBJ_EQUALITY, "grip_"), gripI_ = -1;
  std::vector<int> grip_bodies_(n_grip_bodies_), grip_geoms_(n_grip_bodies_);
  for(int i=0; i<n_grip_bodies_; i++)
  {
    grip_bodies_[i] = mj_name2id(m, mjOBJ_BODY, grip_body_names_[i].c_str());
    grip_geoms_[i] = mj_name2id(m, mjOBJ_GEOM, (grip_body_names_[i]+std::string("_geom")).c_str());
  }

  m->eq_active[lfinger_eqI_] = m->eq_active[rfinger_eqI_] = true; // start locked
  
  // UR5 init
  std_msgs::Float32MultiArray initial_joint_pose_;
  initial_joint_pose_.data = std::vector<float>(UR5_DOF, 0.0);
  initial_joint_pose_.data[1] = -M_PI/2;
  jpos_cb(initial_joint_pose_);
  
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
        ur_nh_.setParam("moving", true);
        // set target position and velocity
        for(int i=0; i<UR5_DOF; i++)
        {          
          rml_i_->TargetPositionVector->VecData[i] = UR5_jpos[UR5_traj_step][i];
          rml_i_->TargetVelocityVector->VecData[i] = UR5_jvel[UR5_traj_step][i];
        }
      }
      else // continue motion
      {
        // update
        int rml_result_ = rml_api_->RMLPosition(*rml_i_,rml_o_, rml_flags_);
        *rml_i_->CurrentPositionVector = *rml_o_->NewPositionVector;
        *rml_i_->CurrentVelocityVector = *rml_o_->NewVelocityVector;
        *rml_i_->CurrentAccelerationVector = *rml_o_->NewAccelerationVector;
        // check if finished
        if(rml_result_ == ReflexxesAPI::RML_FINAL_STATE_REACHED)
          { free(rml_api_); rml_api_ = NULL; UR5_traj_step++; }

        // copy UR5 state to MuJoCo
        for(int i=0; i<UR5_DOF; i++)
        {
          d->qpos[i+UR5_jI_ofs] = rml_o_->NewPositionVector->VecData[i];
          d->qvel[i+UR5_jI_ofs] = rml_o_->NewVelocityVector->VecData[i];
        }
      }

      if(UR5_traj_step == UR5_traj_steps)
        { ur_nh_.setParam("moving", false); UR5_traj_in = false; }
    }
    else
    {
      // remember, no gravity
      for(int i=0; i<UR5_DOF; i++)
      {
        d->qpos[i+UR5_jI_ofs] = UR5_jpos.back()[i];
        d->qvel[i+UR5_jI_ofs] = 0.0;
      }
    }
    
    // gripper control
    if(gripper_state != gripper_in)
    {
      m->eq_active[lfinger_eqI_] = m->eq_active[rfinger_eqI_] = false;
      if(gripper_in) d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = gripper_torque; // close
      else d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = -gripper_torque; // open
        
      gripper_state = gripper_in;
    }
    
    if(gripper_state)
    {
      if(gripI_ == -1)
      {
        gripI_ = checkGrips(gri_l_gI,gri_r_gI, grip_geoms_);
        if(gripI_ != -1)
        {
          // set grip weld
          set_grip_weld_relpose(grip_weldI_, grip_bodies_[gripI_]);
          m->eq_obj2id[grip_weldI_] = grip_bodies_[gripI_];
          m->eq_active[grip_weldI_] = true;
          // lock gripper
          d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = 0.0;
          m->eq_active[lfinger_eqI_] = m->eq_active[rfinger_eqI_] = true;
        }
      }
    }
    else
    {
      if(gripI_ != -1)
        { m->eq_active[grip_weldI_] = false; gripI_ = -1; }
        
      if(std::min(d->qpos[gri_l_jI], d->qpos[gri_r_jI]) <= gripper_driver_min_pos)
      {
        d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = 0.0;
        m->eq_active[lfinger_eqI_] = m->eq_active[rfinger_eqI_] = true;
      }
    }
      
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
  
  ////
  
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
  
  delete rml_api_; delete rml_i_; delete rml_o_;
  
  ////
  
  return 0;
}
