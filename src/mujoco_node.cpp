#include <mujoco_ros/mujoco_node.h>

MujocoNode::MujocoNode()
{
    // constants
    UR5_maxVel =  180 * (M_PI/180); UR5_maxAccel = 40 * (M_PI/180); UR5_maxJerk = 80 * (M_PI/180);
    std::vector<std::vector<double> > UR5_jpos, UR5_jvel; UR5_traj_in = false;
    // gripper control
    gripper_torque = 1.0; gripper_driver_min_pos = 0.05;
    gripper_in = false; gripper_state = false;
  
  

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
    
    reset_srv = nh.advertiseService("reset", &MujocoNode::reset_mujoco_cb, this);
    getpose_srv = nh.advertiseService("get_object_pose", &MujocoNode::getpose_cb, this);
  
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
    // copy part of position and rotation matrix
    res.pos.resize(3); res.rot.resize(9);
    
    if(type_ == mjOBJ_BODY)
    {
      for(int i=0; i<3; i++) res.pos[i] = d->xpos[I_*3+i];
      for(int i=0; i<9; i++) res.rot[i] = d->xmat[I_*9+i];
    }
    else if(type_ == mjOBJ_GEOM)
    {
      for(int i=0; i<3; i++) res.pos[i] = d->geom_xpos[I_*3+i];
      for(int i=0; i<9; i++) res.rot[i] = d->geom_xmat[I_*9+i];
    }
    else if(type_ == mjOBJ_SITE)
    {
      for(int i=0; i<3; i++) res.pos[i] = d->site_xpos[I_*3+i];
      for(int i=0; i<9; i++) res.rot[i] = d->site_xmat[I_*9+i];
    }
  }

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

void MujocoNode::set_grip_weld_relpose(const int& target_bI)
{
  tf::Transform ee_tf_, target_tf_; int eeI_ = m->eq_obj1id[grip_weldI];
  ee_tf_.setOrigin(tf::Vector3(d->xpos[eeI_*3+0], d->xpos[eeI_*3+1], d->xpos[eeI_*3+2]));
  ee_tf_.setRotation(tf::Quaternion(d->xquat[eeI_*4+1], d->xquat[eeI_*4+2], d->xquat[eeI_*4+3], d->xquat[eeI_*4+0]));
  target_tf_.setOrigin(tf::Vector3(d->xpos[target_bI*3+0], d->xpos[target_bI*3+1], d->xpos[target_bI*3+2]));
  target_tf_.setRotation(tf::Quaternion(d->xquat[target_bI*4+1], d->xquat[target_bI*4+2], d->xquat[target_bI*4+3], d->xquat[target_bI*4+0]));
  
  tf::Transform relpose_ = ee_tf_.inverse() * target_tf_;
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
  unsigned char* rgb_buf_raw_ = (unsigned char*)malloc(rgb_buflen_);
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

  std::vector<std::string> grip_body_names_; nh.getParam("grippable_bodies", grip_body_names_);
  size_t n_grip_bodies_ = grip_body_names_.size();
  grippedI = -1; grip_weldI = mj_name2id(m, mjOBJ_EQUALITY, "grip_");
  grippable_bI.resize(n_grip_bodies_); grippable_gI.resize(n_grip_bodies_);
  for(int i=0; i<n_grip_bodies_; i++)
  {
    // bodies for welding, geoms for collision checking
    grippable_bI[i] = mj_name2id(m, mjOBJ_BODY, grip_body_names_[i].c_str());
    grippable_gI[i] = mj_name2id(m, mjOBJ_GEOM, (grip_body_names_[i]+"_geom").c_str());
  }

  // move UR5 to initial pose after locking gripper
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
        }
      }
    }
    else
    {
      if(grippedI != -1)
        { m->eq_active[grip_weldI] = false; grippedI = -1; }
        
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
  // joint indices for 2 other orthogonal axes
  std::string axisA_, axisB_;
  if(req.axis == "x") { axisA_ = "y"; axisB_ = "z"; }
  else if(req.axis == "y") { axisA_ = "x"; axisB_ = "z"; }
  else if(req.axis == "z") { axisA_ = "x"; axisB_ = "y"; }
  else { res.ok = false; return true; }
  
  std::vector<std::string> other_joint_names_(4); 
  other_joint_names_[0] = req.fastener_name + "_t" + axisA_;
  other_joint_names_[1] = req.fastener_name + "_t" + axisB_;
  other_joint_names_[2] = req.fastener_name + "_r" + axisA_;
  other_joint_names_[3] = req.fastener_name + "_r" + axisB_; 
  
  int other_joint_indices_[4];
  for(int i=0; i<4; i++)
  {
    other_joint_indices_[i] = mj_name2id(m, mjOBJ_JOINT, other_joint_names_[i].c_str());
    if(other_joint_indices_[i] == -1) { res.ok = false; return true; }
  }
  
  if(req.attach_flag)
  {
    // append threaded connection
    std::string tn_ = req.fastener_name + "_t" + req.axis,
      rn_ = req.fastener_name + "_r" + req.axis;
    int jtI_ = mj_name2id(m, mjOBJ_JOINT, tn_.c_str()),
      jrI_ = mj_name2id(m, mjOBJ_JOINT, rn_.c_str());
    if((jtI_ != -1) && (jrI_ != -1))
    {
      ThreadedConnection tc_;
      tc_.fastener_name = req.fastener_name; tc_.pitch = req.pitch;
      tc_.tI.p = m->jnt_qposadr[jtI_]; tc_.tI.v = m->jnt_dofadr[jtI_];
      tc_.rI.p = m->jnt_qposadr[jrI_]; tc_.rI.v = m->jnt_dofadr[jrI_];
      threaded_connections.push_back(tc_);

      // other joints: zero velocities and lock
      for(int i=0; i<4; i++)
      {
        d->qvel[m->jnt_dofadr[other_joint_indices_[i]]] = 0.0;
        m->jnt_limited[other_joint_indices_[i]] = true;
      }
    }
    else { res.ok = false; return true; }
  }
  else
  {
    // remove threaded connection
    for(size_t i=0; i<threaded_connections.size(); i++)
      if(threaded_connections[i].fastener_name == req.fastener_name)
        { threaded_connections.erase(threaded_connections.begin()+i); break; }
    
    // unlock other joints
    for(int i=0; i<4; i++)
      m->jnt_limited[other_joint_indices_[i]] = false;
  }
      
  res.ok = true;
  
  return true;
}

void MujocoNode::handle_threaded_connections()
{
  for(size_t i=0; i<threaded_connections.size(); i++)
  {
    d->qvel[threaded_connections[i].tI.v] = d->qvel[threaded_connections[i].rI.v] * threaded_connections[i].pitch;
    d->qpos[threaded_connections[i].tI.p] += d->qvel[threaded_connections[i].tI.v] * FPS_period;
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_node");
  
  MujocoNode mj_node_; mj_node_.loop();
  
  return 0;
}
