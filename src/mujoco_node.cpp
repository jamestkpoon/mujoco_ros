#include <mujoco_ros/mujoco_node.h>

MujocoNode::MujocoNode()
{
    // constants    
    UR5_DOF = 6;
    UR5_maxVel =  180 * (M_PI/180); UR5_maxAccel = 40 * (M_PI/180); UR5_maxJerk = 80 * (M_PI/180);
    std::vector<std::vector<double> > UR5_jpos, UR5_jvel; UR5_traj_in = false;
    // gripper control
    gripper_torque = 1.0; gripper_driver_min_pos = 0.05;
    gripper_in = false; gripper_state = false;
  
  

    // initialize GLFW
    glfwInit();
    window = glfwCreateWindow(640, 480, "Mujoco_UR5", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // activate MuJoCo, initialize
    std::string mujoco_key_; nh_.getParam("key", mujoco_key_);
    mj_activate(mujoco_key_.c_str());
    reset_mujoco();
    
    int fps_; nh_.getParam("FPS", fps_); FPS_period = 1.0 / fps_;
      
      

    // ROS comms  
    ur_nh_ = ros::NodeHandle("ur5");
    jpos_sub_ = ur_nh_.subscribe("command/joint_positions", 1, &MujocoNode::jpos_cb, this);
    gri_sub_ = ur_nh_.subscribe("command/gripper", 1, &MujocoNode::gripper_cb, this);
    
    reset_srv_ = nh_.advertiseService("reset", &MujocoNode::reset_mujoco_cb, this);
}



//// ROS callbacks

void MujocoNode::jpos_cb(const std_msgs::Float32MultiArray& msg)
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

void MujocoNode::gripper_cb(const std_msgs::Bool& msg)
{
  gripper_in = msg.data;
}

bool MujocoNode::reset_mujoco_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  reset_mujoco();
  
  return true;
}



//// gripper functions

bool MujocoNode::checkGrip(const int& lgI, const int& rgI, const int& targetI)
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

int MujocoNode::checkGrips(const int& lgI, const int& rgI, const std::vector<int>& grippable_geomsI)
{
  for(int i=0; i<(int)grippable_geomsI.size(); i++)
    if(checkGrip(lgI,rgI, grippable_geomsI[i])) return i;
    
  return -1;
}

void MujocoNode::set_grip_weld_relpose(const int& grip_weldI, const int& targetI)
{
  tf::Transform ee_tf_, target_tf_; int eeI_ = m->eq_obj1id[grip_weldI];
  ee_tf_.setOrigin(tf::Vector3(d->xpos[eeI_*3+0], d->xpos[eeI_*3+1], d->xpos[eeI_*3+2]));
  ee_tf_.setRotation(tf::Quaternion(d->xquat[eeI_*4+1], d->xquat[eeI_*4+2], d->xquat[eeI_*4+3], d->xquat[eeI_*4+0]));
  target_tf_.setOrigin(tf::Vector3(d->xpos[targetI*3+0], d->xpos[targetI*3+1], d->xpos[targetI*3+2]));
  target_tf_.setRotation(tf::Quaternion(d->xquat[targetI*4+1], d->xquat[targetI*4+2], d->xquat[targetI*4+3], d->xquat[targetI*4+0]));
  
  tf::Transform relpose_ = ee_tf_.inverse() * target_tf_;
  m->eq_data[7*grip_weldI+0] = relpose_.getOrigin()[0];
  m->eq_data[7*grip_weldI+1] = relpose_.getOrigin()[1];
  m->eq_data[7*grip_weldI+2] = relpose_.getOrigin()[2];
  m->eq_data[7*grip_weldI+3] = relpose_.getRotation()[3];
  m->eq_data[7*grip_weldI+4] = relpose_.getRotation()[0];
  m->eq_data[7*grip_weldI+5] = relpose_.getRotation()[1];
  m->eq_data[7*grip_weldI+6] = relpose_.getRotation()[2];
}



//// reset fn, 'main' loop

void MujocoNode::reset_mujoco()
{
  //// MuJoCo
  
  // load model  
  m = NULL; d = NULL;
  std::string mujoco_xml_; nh_.getParam("xml", mujoco_xml_);
  char error[1000];
  m = mj_loadXML(mujoco_xml_.c_str(), NULL, error, 1000);
  d = mj_makeData(m);
  
  // useful indices for control  
  UR5_jI_ofs = mj_name2id(m, mjOBJ_JOINT, "joint1"); // UR5 joints
  gripper_mI_ofs = mj_name2id(m, mjOBJ_ACTUATOR, "close_1"); // gripper motors
  gri_l_jI = mj_name2id(m, mjOBJ_JOINT, "joint7_2"); // gripper drive hinges
  gri_r_jI = mj_name2id(m, mjOBJ_JOINT, "joint7_1");
  gri_l_gI = mj_name2id(m, mjOBJ_GEOM, "left_follower_mesh"); // gripper finger geoms
  gri_r_gI = mj_name2id(m, mjOBJ_GEOM, "right_follower_mesh");
  lfinger_eqI_ = mj_name2id(m, mjOBJ_EQUALITY, "lfinger_lock"); // gripper finger welds
  rfinger_eqI_ = mj_name2id(m, mjOBJ_EQUALITY, "rfinger_lock");
  
  // UR5 init
  std_msgs::Float32MultiArray initial_joint_pose_;
  initial_joint_pose_.data = std::vector<float>(UR5_DOF, 0.0);
  initial_joint_pose_.data[1] = -M_PI/2;
  for(int i=0; i<UR5_DOF; i++)
  {
    d->qpos[i+UR5_jI_ofs] = initial_joint_pose_.data[i];
    d->qvel[i+UR5_jI_ofs] = 0.0;
  }
  jpos_cb(initial_joint_pose_);

  // gripper init
  std::vector<std::string> grip_body_names_; nh_.getParam("grippable_bodies", grip_body_names_);
  int n_grip_bodies_ = (int)grip_body_names_.size();
  gripI_ = -1; grip_weldI_ = mj_name2id(m, mjOBJ_EQUALITY, "grip_");
  grip_bodies_.resize(n_grip_bodies_); grip_geoms_.resize(n_grip_bodies_);
  for(int i=0; i<n_grip_bodies_; i++)
  {
    // bodies for welding, geoms for collision checking
    grip_bodies_[i] = mj_name2id(m, mjOBJ_BODY, grip_body_names_[i].c_str());
    grip_geoms_[i] = mj_name2id(m, mjOBJ_GEOM, (grip_body_names_[i]+std::string("_geom")).c_str());
  }

  m->eq_active[lfinger_eqI_] = m->eq_active[rfinger_eqI_] = true; // start locked  

  // "settle" the simulation by starting at a certain timestamp
  double settle_time_; nh_.getParam("start_time", settle_time_);
  while(d->time < settle_time_) mj_step(m, d);
  
  // RGB over ROS
  ext_cam = nh_.hasParam("ext_cam_name");
  if(ext_cam)
  {
    std::string ext_cam_name; nh_.getParam("ext_cam_name", ext_cam_name);
    ext_camI = mj_name2id(m, mjOBJ_CAMERA, ext_cam_name.c_str());
    rgb_pub_ = ur_nh_.advertise<sensor_msgs::Image>(ext_cam_name+"/rgb", 1);
  }
  


  //// GLFW

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // abstract camera control variables
  glfw_button_left = glfw_button_middle = glfw_button_right = false;
  glfw_lastx = glfw_lasty = 0;

  // set GLFW callbacks
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
  
  
  
  //// Reflexxes
  
  rml_api_ = NULL;
  rml_i_ = new RMLPositionInputParameters(UR5_DOF);
  rml_o_ = new RMLPositionOutputParameters(UR5_DOF);
  
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
}

void MujocoNode::loop()
{  
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
//          d->ctrl[gripper_mI_ofs+0] = d->ctrl[gripper_mI_ofs+1] = 0.0;
//          m->eq_active[lfinger_eqI_] = m->eq_active[rfinger_eqI_] = true;
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
    
    if(ext_cam) // publish RGB
    {
      // render RGB buffer off-screen
      cam.fixedcamid = ext_camI; cam.type = mjCAMERA_FIXED;
      mjr_setBuffer(mjFB_OFFSCREEN, &con);
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);
      int rgb_buflen_ = 3 * viewport.width*viewport.height;
      unsigned char* rgb_buf_ = (unsigned char*)malloc(rgb_buflen_);
      mjr_readPixels(rgb_buf_, NULL, viewport, &con);
      // publish
      sensor_msgs::Image rgb_msg_;
      rgb_msg_.header.stamp = ros::Time::now();
      rgb_msg_.header.frame_id = ext_cam_name;
      rgb_msg_.encoding = "rgb8";
      rgb_msg_.width = viewport.width; rgb_msg_.height = viewport.height;
      rgb_msg_.is_bigendian = true; rgb_msg_.step = 3 * viewport.width;
      rgb_msg_.data = std::vector<unsigned char>(rgb_buf_, rgb_buf_+rgb_buflen_);
      rgb_pub_.publish(rgb_msg_);
      
      // restore rendering to onscreen abstract camera
      cam.fixedcamid = -1; cam.type = mjCAMERA_FREE;
      mjr_setBuffer(mjFB_WINDOW, &con);
    }
    
    // update scene and render for viewport
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }
}

MujocoNode::~MujocoNode()
{
  //// MuJoCo/Reflexxes cleanup

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deactivate();

  // close window, stop GLFW
  if(!glfwWindowShouldClose(window)) glfwSetWindowShouldClose(window, GLFW_TRUE);
  glfwTerminate();
  
  // Reflexxes resources
  delete rml_api_; delete rml_i_; delete rml_o_;
}



//// GLFW mouse callbacks

void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    glfw_button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    glfw_button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    glfw_button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &glfw_lastx, &glfw_lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !glfw_button_left && !glfw_button_middle && !glfw_button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - glfw_lastx;
    double dy = ypos - glfw_lasty;
    glfw_lastx = xpos;
    glfw_lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( glfw_button_left )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( glfw_button_middle )
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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_node");
  
  MujocoNode mj_node_; mj_node_.loop();
  
  return 0;
}
