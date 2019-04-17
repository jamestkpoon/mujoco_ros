#include <mujoco_ros/mujoco_node.h>

MujocoNode::MujocoNode()
{
    // UR5 constants
    UR5_maxVel =  180 * (M_PI/180); UR5_maxAccel = 40 * (M_PI/180); UR5_maxJerk = 80 * (M_PI/180);
    // gripper constants
    gripper_torque = 1.0; gripper_driver_min_pos = 0.05;

  

    // initialize GLFW
    glfwInit();
    window = glfwCreateWindow(GLFW_W, GLFW_H, "Mujoco_UR5", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // activate MuJoCo, initialize
    std::string mujoco_key_; nh.getParam("key", mujoco_key_);
    mj_activate(mujoco_key_.c_str());
    reset_mujoco();
    
    int fps_; nh.getParam("FPS", fps_); FPS_period = 1.0 / fps_;
      
      

    // ROS comms  
    ur_nh = ros::NodeHandle("ur5");
    jpos_sub = ur_nh.subscribe("command/joint_positions", 1, &MujocoNode::jpos_cb, this);
    gri_sub = ur_nh.subscribe("command/gripper", 1, &MujocoNode::gripper_cb, this);
    
    jstate_pub = ur_nh.advertise<std_msgs::Float32MultiArray>("sensors/joint_state", 1);
    
    reset_srv = nh.advertiseService("reset", &MujocoNode::reset_mujoco_cb, this);
//    getpose_srv = nh.advertiseService("get_object_pose", &MujocoNode::getpose_cb, this);
    get_brelpose_srv = nh.advertiseService("get_relative_pose_bodies", &MujocoNode::get_brelpose_cb, this);
  
    // stream camera data over ROS
    ext_cam = nh.hasParam("ext_cam_name");
    if(ext_cam)
    {
      nh.getParam("ext_cam_name", ext_cam_name);
      ext_camI = mj_name2id(m, mjOBJ_CAMERA, ext_cam_name.c_str());
      ext_cam_pub = nh.advertise<sensor_msgs::Image>(ext_cam_name+"/rgb", 1);
    }
    
    
    
    // threaded manip
    threadlock_srv = nh.advertiseService("thread_lock", &MujocoNode::threadlock_cb, this);
   
    // randomization
    randtex_srv = nh.advertiseService("rand/textural", &MujocoNode::randomize_textural_cb, this);
    randphys_srv = nh.advertiseService("rand/physical", &MujocoNode::randomize_physical_cb, this);
}



//// general ROS callbacks

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

bool MujocoNode::getpose_cb(mujoco_ros::GetPose::Request& req, mujoco_ros::GetPose::Response& res)
{
  int type_ = -1;
  if(req.type == "body") type_ = mjOBJ_BODY;
  else if(req.type == "geom") type_ = mjOBJ_GEOM;
  else if(req.type == "site") type_ = mjOBJ_SITE;
  else return true;
  
  int I_ = mj_name2id(m, type_, req.name.c_str());
  if(I_ != -1)
  {
    // copy sub-parts of position and rotation matrices
    res.pos.resize(3); res.rot.resize(9);
    
    if(type_ == mjOBJ_BODY)
    {
      for(int i=0; i<3; i++) res.pos[i] = d->xpos[3*I_+i];
      for(int i=0; i<9; i++) res.rot[i] = d->xmat[9*I_+i];
    }
    else if(type_ == mjOBJ_GEOM)
    {
      for(int i=0; i<3; i++) res.pos[i] = d->geom_xpos[3*I_+i];
      for(int i=0; i<9; i++) res.rot[i] = d->geom_xmat[9*I_+i];
    }
    else if(type_ == mjOBJ_SITE)
    {
      for(int i=0; i<3; i++) res.pos[i] = d->site_xpos[3*I_+i];
      for(int i=0; i<9; i++) res.rot[i] = d->site_xmat[9*I_+i];
    }
  }
  else { res.pos.clear(); res.rot.clear(); }

  return true;
}

bool MujocoNode::get_brelpose_cb(mujoco_ros::GetRelativePoseBodies::Request& req,
  mujoco_ros::GetRelativePoseBodies::Response& res)
{
  int baI_ = mj_name2id(m, mjOBJ_BODY, req.bodyA_name.c_str()),
    bbI_ = mj_name2id(m, mjOBJ_BODY, req.bodyB_name.c_str());

  if((baI_ != -1) && (bbI_ != -1))
  {
    tf::Transform tfo_; get_relpose(tfo_, baI_,bbI_); tf::poseTFToMsg(tfo_, res.relpose);
    res.ok = true;
  }
  else res.ok = false;

  return true;
}



//// gripper functions

bool MujocoNode::checkGrip(const int& target_gI)
{
  bool lcon_ = false, rcon_ = false;
  for(int i=0; i<d->ncon; i++)
  {
    if(!lcon_ && (d->contact[i].geom1 == gri_l_gI || d->contact[i].geom2 == gri_l_gI))
      lcon_ = (d->contact[i].geom1 == target_gI || d->contact[i].geom2 == target_gI);
    if(!rcon_ && (d->contact[i].geom1 == gri_r_gI || d->contact[i].geom2 == gri_r_gI))
      rcon_ = (d->contact[i].geom1 == target_gI || d->contact[i].geom2 == target_gI);
  }
  
  return (lcon_ && rcon_);
}

int MujocoNode::checkGrips()
{
  for(int i=0; i<(int)grippable_gI.size(); i++)
    if(checkGrip(grippable_gI[i])) return i;
    
  return -1;
}

void MujocoNode::xpose_to_tf(tf::Transform& tf_out, const int& bI)
{
  tf_out.setOrigin(tf::Vector3(d->xpos[3*bI+0], d->xpos[3*bI+1], d->xpos[3*bI+2]));
  tf_out.setRotation(tf::Quaternion(d->xquat[4*bI+1], d->xquat[4*bI+2], d->xquat[4*bI+3], d->xquat[4*bI+0]));
}

void MujocoNode::get_relpose(tf::Transform& tf_out, const int& baI, const int& bbI)
{
  tf::Transform ba_tf_, bb_tf_; xpose_to_tf(ba_tf_, baI); xpose_to_tf(bb_tf_, bbI);
  tf_out = ba_tf_.inverse() * bb_tf_;
}

void MujocoNode::set_grip_weld_relpose(const int& target_bI)
{
  tf::Transform relpose_; get_relpose(relpose_, m->eq_obj1id[grip_weldI], target_bI);

  m->eq_data[7*grip_weldI+0] = relpose_.getOrigin()[0];
  m->eq_data[7*grip_weldI+1] = relpose_.getOrigin()[1];
  m->eq_data[7*grip_weldI+2] = relpose_.getOrigin()[2];
  m->eq_data[7*grip_weldI+3] = relpose_.getRotation()[3];
  m->eq_data[7*grip_weldI+4] = relpose_.getRotation()[0];
  m->eq_data[7*grip_weldI+5] = relpose_.getRotation()[1];
  m->eq_data[7*grip_weldI+6] = relpose_.getRotation()[2];
}



//// streaming RGB camera

void MujocoNode::fill_rgb_image_msg(sensor_msgs::Image& msg,
  const unsigned char* buf, const int& buf_sz)
{
  // other data fields
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = ext_cam_name;
  msg.encoding = "rgb8";
  msg.width = GLFW_W; msg.height = GLFW_H;
  msg.step = msg.width * GLFW_C;
  
  // copy
//  msg.data = std::vector<unsigned char>(buf, buf+buf_sz);

  // horizontal flip
  msg.data.resize(buf_sz); int iI_ = 0;
  for(int v=0; v<msg.height; v++)
  {
    int oI_ = (v+1)*msg.step - GLFW_C;
    for(int u=0; u<msg.width; u++)
    {
      for(int c=0; c<GLFW_C; c++) msg.data[oI_++] = buf[iI_++];
      oI_ -= 2*GLFW_C; // shift output index back to start of preceding element
    }
  }
}

void MujocoNode::publish_cam_rgb(mjrRect& viewport)
{
  // render RGB off-screen
  cam.fixedcamid = ext_camI; cam.type = mjCAMERA_FIXED;
  mjr_setBuffer(mjFB_OFFSCREEN, &con);
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);
  // read buffer
  int rgb_buflen_ = GLFW_W*GLFW_H * GLFW_C;
  unsigned char rgb_buf_raw_[rgb_buflen_];
  mjr_readPixels(rgb_buf_raw_, NULL, viewport, &con);
  
  // publish
  sensor_msgs::Image rgb_msg_;
  fill_rgb_image_msg(rgb_msg_, rgb_buf_raw_,rgb_buflen_);
  ext_cam_pub.publish(rgb_msg_);
  
  // restore rendering to onscreen abstract camera
  cam.fixedcamid = -1; cam.type = mjCAMERA_FREE;
  mjr_setBuffer(mjFB_WINDOW, &con);
}



//// reset fn, 'main' loop, deconstructor

void MujocoNode::reset_mujoco()
{
  //// MuJoCo
  
  // load model  
  m = NULL; d = NULL;
  std::string mujoco_xml_; nh.getParam("xml", mujoco_xml_);
  char error[1000];
  m = mj_loadXML(mujoco_xml_.c_str(), NULL, error, 1000);
  d = mj_makeData(m);
  
  // UR5 indexing
  UR5_jI.resize(UR5_DOF);
  for(int i=0; i<UR5_DOF; i++)
  {
    std::string jn_ = "joint" + boost::lexical_cast<std::string>(i+1);
    int i_ = mj_name2id(m, mjOBJ_JOINT, jn_.c_str());
    UR5_jI[i].p = m->jnt_qposadr[i_]; UR5_jI[i].v = m->jnt_dofadr[i_];
  }
  
  // gripper indexing
  gri_jI.resize(2);
  gri_jI[0].p = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint7_1")];
  gri_jI[1].p = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint7_2")];
  gripper_m1I = mj_name2id(m, mjOBJ_ACTUATOR, "gripper_close_1"); // gripper motors
  gripper_m2I = mj_name2id(m, mjOBJ_ACTUATOR, "gripper_close_2");
  gri_l_gI = mj_name2id(m, mjOBJ_GEOM, "left_follower_mesh"); // gripper finger geoms
  gri_r_gI = mj_name2id(m, mjOBJ_GEOM, "right_follower_mesh");
  lfinger_eqI = mj_name2id(m, mjOBJ_EQUALITY, "lfinger_lock"); // gripper finger welds
  rfinger_eqI = mj_name2id(m, mjOBJ_EQUALITY, "rfinger_lock");

  nh.getParam("grippable_bodies", grippable_body_names);
  size_t n_grip_bodies_ = grippable_body_names.size();
  grippedI = -1; grip_weldI = mj_name2id(m, mjOBJ_EQUALITY, "grip_");
  grippable_bI.resize(n_grip_bodies_); grippable_gI.resize(n_grip_bodies_);
  for(size_t i=0; i<n_grip_bodies_; i++)
  {
    // bodies for welding, geoms for collision checking
    grippable_bI[i] = mj_name2id(m, mjOBJ_BODY, grippable_body_names[i].c_str());
    grippable_gI[i] = mj_name2id(m, mjOBJ_GEOM, (grippable_body_names[i]+"_geom").c_str());
  }
  ur_nh.setParam("gripped_object", "");

  // move UR5 to initial pose after locking gripper
  UR5_traj_in = false; gripper_in = false; gripper_state = false;
  m->eq_active[lfinger_eqI] = m->eq_active[rfinger_eqI] = true; // lock gripper
  std_msgs::Float32MultiArray initial_joint_pose_;
  initial_joint_pose_.data = std::vector<float>(UR5_DOF, 0.0);
  initial_joint_pose_.data[1] = -M_PI/2;
  for(int i=0; i<UR5_DOF; i++)
  {
    d->qpos[UR5_jI[i].p] = initial_joint_pose_.data[i];
    d->qvel[UR5_jI[i].v] = 0.0;
  }
  jpos_cb(initial_joint_pose_);

  // "settle" the simulation
  double settle_time_; nh.getParam("start_time", settle_time_);
  while(d->time < settle_time_) mj_step(m, d);
 


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
  
  rml_api = NULL;
  rml_i = new RMLPositionInputParameters(UR5_DOF);
  rml_o = new RMLPositionOutputParameters(UR5_DOF);
  
  for(int i=0; i<UR5_DOF; i++)
  {
    rml_i->CurrentPositionVector->VecData[i] = d->qpos[UR5_jI[i].p];
    rml_i->CurrentVelocityVector->VecData[i] = d->qvel[UR5_jI[i].v];
    rml_i->CurrentAccelerationVector->VecData[i] = 0.0;
            
    rml_i->MaxVelocityVector->VecData[i] = UR5_maxVel;
    rml_i->MaxAccelerationVector->VecData[i] = UR5_maxAccel;
    rml_i->MaxJerkVector->VecData[i] = UR5_maxJerk;
    
    rml_i->SelectionVector->VecData[i] = true;
  }
  
  ur_nh.setParam("moving", false);
  
  
  
  //// threaded manip
  threaded_connections.clear();
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
      if(rml_api == NULL) // start new motion
      {
        rml_api = new ReflexxesAPI(UR5_DOF, FPS_period);
        ur_nh.setParam("moving", true);
        // set target position and velocity
        for(int i=0; i<UR5_DOF; i++)
        {          
          rml_i->TargetPositionVector->VecData[i] = UR5_jpos[UR5_traj_step][i];
          rml_i->TargetVelocityVector->VecData[i] = UR5_jvel[UR5_traj_step][i];
        }
      }
      else // continue motion
      {
        // update
        int rml_result_ = rml_api->RMLPosition(*rml_i,rml_o, rml_flags);
        *rml_i->CurrentPositionVector = *rml_o->NewPositionVector;
        *rml_i->CurrentVelocityVector = *rml_o->NewVelocityVector;
        *rml_i->CurrentAccelerationVector = *rml_o->NewAccelerationVector;
        // check if finished
        if(rml_result_ == ReflexxesAPI::RML_FINAL_STATE_REACHED)
          { free(rml_api); rml_api = NULL; UR5_traj_step++; }

        // copy UR5 state to MuJoCo
        for(int i=0; i<UR5_DOF; i++)
        {
          d->qpos[UR5_jI[i].p] = rml_o->NewPositionVector->VecData[i];
          d->qvel[UR5_jI[i].v] = rml_o->NewVelocityVector->VecData[i];
        }
      }

      if(UR5_traj_step == UR5_traj_steps)
        { ur_nh.setParam("moving", false); UR5_traj_in = false; }
    }
    else
    {
      // lock joints for gravity compensation
      for(int i=0; i<UR5_DOF; i++)
      {
        d->qpos[UR5_jI[i].p] = UR5_jpos.back()[i];
        d->qvel[UR5_jI[i].v] = 0.0;
      }
    }
    
    // publish joint state
    std_msgs::Float32MultiArray UR5_jstate_out_;
    UR5_jstate_out_.data.resize(UR5_DOF*2);
    for(int i=0; i<UR5_DOF; i++)
    {
      UR5_jstate_out_.data[i] = d->qpos[UR5_jI[i].p];
      UR5_jstate_out_.data[UR5_DOF+i] = d->qvel[UR5_jI[i].v];
    }
    jstate_pub.publish(UR5_jstate_out_);
    
    // gripper control
    if(gripper_state != gripper_in)
    {
      m->eq_active[lfinger_eqI] = m->eq_active[rfinger_eqI] = false;
      if(gripper_in) d->ctrl[gripper_m1I] = d->ctrl[gripper_m2I] = gripper_torque; // close
      else d->ctrl[gripper_m1I] = d->ctrl[gripper_m2I] = -gripper_torque; // open
        
      gripper_state = gripper_in;
    }
    
    if(gripper_state)
    {
      if(grippedI == -1)
      {
        grippedI = checkGrips();
        if(grippedI != -1)
        {
          // set grip weld
          set_grip_weld_relpose(grippable_bI[grippedI]);
          m->eq_obj2id[grip_weldI] = grippable_bI[grippedI];
          m->eq_active[grip_weldI] = true;
          // set gripped object rosparam
          ur_nh.setParam("gripped_object", grippable_body_names[grippedI]);
        }
      }
    }
    else
    {
      if(grippedI != -1)
      {
        m->eq_active[grip_weldI] = false;
        grippedI = -1; ur_nh.setParam("gripped_object", "");
      }
        
      if(std::min(d->qpos[gri_jI[0].p], d->qpos[gri_jI[1].p]) <= gripper_driver_min_pos)
      {
        d->ctrl[gripper_m1I] = d->ctrl[gripper_m2I] = 0.0;
        m->eq_active[lfinger_eqI] = m->eq_active[rfinger_eqI] = true;
      }
    }
    
    // threaded manip
    handle_threaded_connections();
      
    // update MuJoCo
    mjtNum simstart = d->time;
    while( d->time - simstart < FPS_period ) mj_step(m, d);    
    
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    
    if(ext_cam) publish_cam_rgb(viewport); // publish RGB
    
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
  delete rml_api; delete rml_i; delete rml_o;
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



//// threaded manip

bool MujocoNode::threadlock_cb(mujoco_ros::ThreadLock::Request& req, mujoco_ros::ThreadLock::Response& res)
{
  if(req.attach_flag)
  {
    // all 6 joint names
    std::string axisA_, axisB_;
    if(req.axis == "x") { axisA_ = "y"; axisB_ = "z"; }
    else if(req.axis == "y") { axisA_ = "x"; axisB_ = "z"; }
    else if(req.axis == "z") { axisA_ = "x"; axisB_ = "y"; }
    else { res.ok = false; return true; }
    
    std::vector<std::string> joint_names_(6);
    joint_names_[0] = req.fastener_name + "_t" + req.axis;
    joint_names_[1] = req.fastener_name + "_r" + req.axis;
    joint_names_[2] = req.fastener_name + "_t" + axisA_;
    joint_names_[3] = req.fastener_name + "_r" + axisA_;
    joint_names_[4] = req.fastener_name + "_t" + axisB_;
    joint_names_[5] = req.fastener_name + "_r" + axisB_;
    
    // populate potential connection item
    ThreadedConnection tc_;
    tc_.fastener_name = req.fastener_name; tc_.pitch = req.pitch;
    for(int i=0; i<6; i++)
    {
      tc_.jI[i].I = mj_name2id(m, mjOBJ_JOINT, joint_names_[i].c_str());
      if(tc_.jI[i].I != -1)
      {
        tc_.jI[i].p = m->jnt_qposadr[tc_.jI[i].I];
        tc_.jI[i].v = m->jnt_dofadr[tc_.jI[i].I];
      }
    }
    
    // valid if both joints exist for threading axis
    if((tc_.jI[0].I != -1) && (tc_.jI[1].I != -1))
    {
      // constrain other joints, where present
      for(int i=2; i<6; i++)
        if(tc_.jI[i].I != -1)
        {
          m->jnt_range[2*tc_.jI[i].I+0] = d->qpos[tc_.jI[i].p] - JNT_LOCK_TOL;
          m->jnt_range[2*tc_.jI[i].I+1] = d->qpos[tc_.jI[i].p] + JNT_LOCK_TOL;
          m->jnt_limited[tc_.jI[i].I] = true; d->qvel[tc_.jI[i].v] = 0.0;
        }
      // append connection, return
      threaded_connections.push_back(tc_);
      res.ok = true; return true;
    }
    else { res.ok = false; return true; }
  }
  else
  {
    for(size_t i=0; i<threaded_connections.size(); i++)
      if(threaded_connections[i].fastener_name == req.fastener_name)
      {
        // free other joints, where present
        for(int j=2; j<6; j++)
          if(threaded_connections[i].jI[j].I != -1)
            m->jnt_limited[threaded_connections[i].jI[j].I] = false;
        // erase connection, return
        threaded_connections.erase(threaded_connections.begin()+i);
        res.ok = true; return true;
      }
    res.ok = false; return true;
  }
}

void MujocoNode::handle_threaded_connections()
{
  for(size_t i=0; i<threaded_connections.size(); i++)
  {
    d->qvel[threaded_connections[i].jI[0].v] = d->qvel[threaded_connections[i].jI[1].v] * threaded_connections[i].pitch;
    d->qpos[threaded_connections[i].jI[0].p] += d->qvel[threaded_connections[i].jI[0].v] * FPS_period;
  }
}



// randomization

bool MujocoNode::randomize_textural_cb(mujoco_ros::RandomizeTexturalAttribute::Request& req, mujoco_ros::RandomizeTexturalAttribute::Response& res)
{
  res.ok.clear();
  
  if(req.noise.size() == 8*req.material_names.size())
    for(int i=0; i<(int)req.material_names.size(); i++)
    {
      int matI_ = mj_name2id(m, mjOBJ_MATERIAL, req.material_names[i].c_str());
      if(matI_ != -1)
      {
        // req.noise: [ emission*1, specular*1, shininess*1, reflectance*1, rgba*4 ] * N_material_names
        m->mat_emission[matI_] = randomize_with_noise_01(m->mat_emission[matI_], req.noise[8*i+0]);
        m->mat_specular[matI_] = randomize_with_noise_01(m->mat_specular[matI_], req.noise[8*i+1]);
        m->mat_shininess[matI_] = randomize_with_noise_01(m->mat_shininess[matI_], req.noise[8*i+2]);
        m->mat_reflectance[matI_] = randomize_with_noise_01(m->mat_reflectance[matI_], req.noise[8*i+3]);
        for(int i=0; i<4; i++)
          m->mat_rgba[4*matI_+i] = randomize_with_noise_01(m->mat_rgba[4*matI_+i], req.noise[8*i+4+i]);
          
        res.ok.push_back(true);
      }
      else res.ok.push_back(false);
    }
  
  return true;
}

double MujocoNode::randomize_with_noise_01(const double& mean, const double& noise)
{
  if(mean == 0.0) return noise * rand_01();
  else if(mean == 1.0) return 1.0 - (noise * rand_01());
  else return std::max(0.0, std::min(mean + (noise * rand_pm1()), 1.0));
}

bool MujocoNode::randomize_physical_cb(mujoco_ros::RandomizePhysicalAttribute::Request& req, mujoco_ros::RandomizePhysicalAttribute::Response& res)
{
  res.ok.clear();

  
  if(req.noise.size() == 6*req.body_names.size())
    for(int i=0; i<(int)req.body_names.size(); i++)
    {
      int bI_ = mj_name2id(m, mjOBJ_BODY, req.body_names[i].c_str());
      if(bI_ != -1)
      {      
        // req.noise: [ translation*3, euler*3 ] * N_body_names        
        JointIndex jI_[6]; bool jI_ok_ = true;
        jI_[0].I = mj_name2id(m, mjOBJ_JOINT, (req.body_names[i]+"_tx").c_str());
        jI_[1].I = mj_name2id(m, mjOBJ_JOINT, (req.body_names[i]+"_ty").c_str());
        jI_[2].I = mj_name2id(m, mjOBJ_JOINT, (req.body_names[i]+"_tz").c_str());
        jI_[3].I = mj_name2id(m, mjOBJ_JOINT, (req.body_names[i]+"_rx").c_str());
        jI_[4].I = mj_name2id(m, mjOBJ_JOINT, (req.body_names[i]+"_ry").c_str());
        jI_[5].I = mj_name2id(m, mjOBJ_JOINT, (req.body_names[i]+"_rz").c_str());
        for(int j=0; j<6; j++)
          if(jI_[j].I != -1)
          {
            jI_[j].p = m->jnt_qposadr[jI_[j].I];
            jI_[j].v = m->jnt_dofadr[jI_[j].I];
          }
          else { jI_ok_ = false; break; }
         
        if(jI_ok_)
        {
          for(int j=0; j<6; j++)
          {
            d->qpos[jI_[j].p] += req.noise[6*i+j] * rand_pm1();
            d->qvel[jI_[j].v] = 0.0; 
          }
        }
        
        res.ok.push_back(jI_ok_);
      }
      else res.ok.push_back(false);
    }
  
  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_node");
  
  MujocoNode mj_node_; mj_node_.loop();
  
  return 0;
}
