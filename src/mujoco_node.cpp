#include <mujoco_ros/mujoco_node.h>

MujocoNode::MujocoNode()
{
  // simulation frequency
  int fps_; nh.getParam("FPS", fps_); mujoco_dt = 1.0 / fps_;
  
  // UR5 + gripper
  robot_nh = ros::NodeHandle("ur5");
  ur5 = new UR5(robot_nh, mujoco_dt);
  gripper = new Gripper(robot_nh);
  
  // other "modules"
  fb_tracker = new FreeBodyTracker(mujoco_dt);
  tb_locker = new ThreadedBodyLocker(nh);
  
  

  // initialize GLFW
  glfwInit();
  window = glfwCreateWindow(GLFW_W, GLFW_H, "MujocoROS", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glfw_button_left = glfw_button_middle = glfw_button_right = false;
  glfw_lastx = glfw_lasty = 0;

  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);



  // activate MuJoCo, initialize
  std::string mujoco_key_; nh.getParam("key", mujoco_key_);
  mj_activate(mujoco_key_.c_str());
  reset();     
  

  
  reset_srv = nh.advertiseService("reset", &MujocoNode::reset_mujoco_cb, this);
//    getpose_srv = nh.advertiseService("get_object_pose", &MujocoNode::getpose_cb, this);
  get_brelpose_srv = nh.advertiseService("get_relative_pose_bodies", &MujocoNode::get_brelpose_cb, this);

  // publish camera data to ROS
  ext_cam = nh.hasParam("ext_cam_name");
  if(ext_cam)
  {
    nh.getParam("ext_cam_name", ext_cam_name);
    ext_camI = mj_name2id(m, mjOBJ_CAMERA, ext_cam_name.c_str());
    ext_cam_pub = nh.advertise<sensor_msgs::Image>(ext_cam_name+"/rgb", 1);
  }
  
  
 
  // randomization
  randtex_srv = nh.advertiseService("rand/textural", &MujocoNode::randomize_textural_cb, this);
  randphys_srv = nh.advertiseService("rand/physical", &MujocoNode::randomize_physical_cb, this);
}

MujocoNode::~MujocoNode()
{  
  free_mujoco_memory();

  // deactivate Mujoco session
  mj_deactivate();

  // close window, stop GLFW
  if(!glfwWindowShouldClose(window)) glfwSetWindowShouldClose(window, GLFW_TRUE);
  glfwTerminate();
 
  // free other things
  delete ur5; delete gripper;
  delete fb_tracker;
  delete tb_locker;
}



void MujocoNode::free_mujoco_memory()
{
  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);
}

void MujocoNode::get_indexing()
{  


  
  
  

}

void MujocoNode::reset()
{
  //// MuJoCo
  
  // load model  
  std::string mujoco_xml_; nh.getParam("xml", mujoco_xml_);
  char error[1000];  
  m = mj_loadXML(mujoco_xml_.c_str(), NULL, error, 1000);
  d = mj_makeData(m);

  // create scene and context
  mjv_defaultCamera(&cam); mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);  
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // step the simulation to start time
  
  mj_step(m, d); // at least once ?
  
  double settle_time_; nh.getParam("start_time", settle_time_);
  while(d->time < settle_time_) mj_step(m, d); 
  
  
  
  //// UR5 + gripper
  
  // UR5 indexing
  std::vector<std::string> ur5_joint_names_(6);
  for(int i=0; i<6; i++)
  {
    ur5_joint_names_[i] = "joint" + boost::lexical_cast<std::string>(i+1);
  }
  
  // gripper indexing
  std::vector<GripperFinger> gf_(2);
  gf_[0].joint_posI = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint7_1")]; // joint pos
  gf_[1].joint_posI = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint7_2")];
  gf_[0].motorI = mj_name2id(m, mjOBJ_ACTUATOR, "gripper_close_1"); // motors
  gf_[1].motorI = mj_name2id(m, mjOBJ_ACTUATOR, "gripper_close_2");
  gf_[0].ftip_geomI = mj_name2id(m, mjOBJ_GEOM, "left_follower_mesh"); // fingertip geoms
  gf_[1].ftip_geomI = mj_name2id(m, mjOBJ_GEOM, "right_follower_mesh");
  gf_[0].lock_weldI = mj_name2id(m, mjOBJ_EQUALITY, "lfinger_lock"); // fingertip welds
  gf_[1].lock_weldI = mj_name2id(m, mjOBJ_EQUALITY, "rfinger_lock");
  
  int grasp_eqI_ = mj_name2id(m, mjOBJ_EQUALITY, "grasp_");
  std::vector<std::string> graspable_body_names_;
  nh.getParam("graspable_bodies", graspable_body_names_);
  std::vector<int> graspable_geomsI_;
  for(int i=0; i<(int)graspable_body_names_.size(); i++)
  {
    int bI_ = mj_name2id(m, mjOBJ_BODY, graspable_body_names_[i].c_str());
    if(bI_ != -1)
      for(int i=0; i<m->body_geomnum[bI_]; i++)
        graspable_geomsI_.push_back(m->body_geomadr[bI_]+i);
  }
  
  // init
  ur5->init(m,d, robot_nh, ur5_joint_names_);
  gripper->init(m,d, robot_nh, gf_,grasp_eqI_,graspable_geomsI_);
    
  

  //// other "modules"
  
  // free body joint tracker
  std::vector<std::string> free_body_names_; nh.getParam("free_bodies", free_body_names_);
  fb_tracker->init(m,d, free_body_names_);
  
  // threaded object locking handler
  tb_locker->init();
  
  // randomization
  rand_child_fix.clear(); rand_proc_now = false;


}

void MujocoNode::loop()
{
  while(!glfwWindowShouldClose(window) && ros::ok())
  {
    // ROS comms
    ros::spinOnce();    
    
    // UR5 + gripper
    ur5->proc(m,d, robot_nh);
    gripper->proc(m,d, robot_nh);    
    
    // other "modules"
    tb_locker->proc(m,d, fb_tracker);
    fb_tracker->proc(m,d); // do last
    
    
      
    //// update sim
    
    mjtNum simstart = d->time;
    while( d->time - simstart < mujoco_dt ) mj_step(m, d);    
    
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    
    // publish camera data to ROS
    if(ext_cam) publish_cam_rgb(viewport);
    
    // update scene and render for viewport
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();   
  }
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
        m->mat_emission[matI_] = rand_clip(m->mat_emission[matI_], req.noise[8*i+0], 0.0,1.0);
        m->mat_specular[matI_] = rand_clip(m->mat_specular[matI_], req.noise[8*i+1], 0.0,1.0);
        m->mat_shininess[matI_] = rand_clip(m->mat_shininess[matI_], req.noise[8*i+2], 0.0,1.0);
        m->mat_reflectance[matI_] = rand_clip(m->mat_reflectance[matI_], req.noise[8*i+3], 0.0,1.0);
        for(int i=0; i<4; i++)
          m->mat_rgba[4*matI_+i] = rand_clip(m->mat_rgba[4*matI_+i], req.noise[8*i+4+i], 0.0,1.0);
          
        res.ok.push_back(true);
      }
      else res.ok.push_back(false);
    }
  
  return true;
}

double MujocoNode::rand_clip(const double mean, const double noise, const double lb, const double ub)
{
  if(mean == lb) return lb + (noise * rand_01());
  else if(mean == ub) return ub - (noise * rand_01());
  else return std::max(lb, std::min(mean + (noise * rand_pm1()), ub));
}

bool MujocoNode::randomize_physical_cb(mujoco_ros::RandomizePhysicalAttribute::Request& req, mujoco_ros::RandomizePhysicalAttribute::Response& res)
{
  res.ok.clear();
  
  // get body indices, check joint data lengths
  int nb_ = (int)req.body_names.size(), bI_[nb_], nj_cumsum_=0;
  for(int i=0; i<nb_; i++)
  {
    bI_[i] = mj_name2id(m, mjOBJ_BODY, req.body_names[i].c_str());
    if(bI_[i] != -1) nj_cumsum_ += m->body_jntnum[bI_[i]];
    else return true; 
  }
  
  // randomize if total lengths match
  if((nj_cumsum_ == (int)req.noise.size()) && (nb_ == (int)req.hold_6dof_children.size()))
  {
    // bodies
    int noiseI_ = 0;
    for(int i=0; i<nb_; i++)
    {
      if(m->body_jntnum[bI_[i]] == 0) res.ok.push_back(false);
      else
      {
        res.ok.push_back(true);
        JointIndex jI_; jI_.I = m->body_jntadr[bI_[i]];
        for(int j=0; j<m->body_jntnum[bI_[i]]; j++)
        {
          jI_.p = m->jnt_qposadr[jI_.I+j]; jI_.v = m->jnt_dofadr[jI_.I+j];
          d->qpos[jI_.p] += req.noise[noiseI_++] * rand_pm1();
          d->qvel[jI_.v] = 0.0;
        }
      }
    }
    
    // children, assumes 6 joints [ tx,ty,tz, rx,ry,rz ]
    rand_child_fix.clear();
    for(int p=0; p<nb_; p++)
      if(req.hold_6dof_children[p])
      {
        for(int b=0; b<m->nbody; b++)
          if(rand_childOK(b, bI_[p]))
          {
            PCtf pc_;
            pc_.p_bI = m->body_parentid[b]; // parent body index
            xpose_to_tf(m,d, pc_.world_c, b); // child pose wrt world
            for(int j=0; j<6; j++) // child joint indices
            {
              pc_.jpI[j] = m->jnt_qposadr[m->body_jntadr[b]+j];
              pc_.jvI[j] = m->jnt_dofadr[m->body_jntadr[b]+j];
            }
            
            rand_child_fix.push_back(pc_);
          }
      }
  }
  
  return true;
}

bool MujocoNode::rand_childOK(const int cI, const int pI)
{
  bool lineage_ = false; int pI_ = cI;
  while(pI_ != 0)
  {
    pI_ = m->body_parentid[pI_];
    if(pI_ == pI) { lineage_ = true; break; }
  }
  
  return (lineage_ && (m->body_jntnum[cI] == 6));
}

void MujocoNode::handle_randomization()
{
  if(!rand_child_fix.empty())
    if(rand_proc_now)
    {
      for(size_t i=0; i<rand_child_fix.size(); i++)
        rand_child_pose_fix(rand_child_fix[i]);
      rand_child_fix.clear(); rand_proc_now = false;
    }
    else rand_proc_now = true;
}

void MujocoNode::rand_child_pose_fix(PCtf& pc)
{
  // relative p->c pose
  xpose_to_tf(m,d, pc.world_p, pc.p_bI);
  tf::Transform rp_tf_ = pc.world_p.inverse() * pc.world_c;
  // euler rotation
  tf::Matrix3x3 m_(rp_tf_.getRotation());
  double eul_[3]; m_.getRPY(eul_[0], eul_[1], eul_[2]);
  
  // adjust child joint positions
  for(int i=0; i<3; i++)
  {
    d->qpos[pc.jpI[i]] = rp_tf_.getOrigin()[i]; // slide joint
    d->qpos[pc.jpI[3+i]] = eul_[i]; // hinge joint
  }
  // zero child joint velocities
  for(int j=0; j<6; j++) d->qvel[pc.jvI[j]] = 0.0;
}



//// ROS

bool MujocoNode::reset_mujoco_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  free_mujoco_memory(); reset();
  
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
    tf::Transform tfo_; xpose_to_tf_rel(m,d, tfo_, baI_,bbI_); tf::poseTFToMsg(tfo_, res.relpose);
    res.ok = true;
  }
  else res.ok = false;

  return true;
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

void MujocoNode::fill_rgb_image_msg(sensor_msgs::Image& msg,
  const unsigned char* buf, const int buf_sz)
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



//// main

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_node");
  
  MujocoNode mj_node_; mj_node_.loop();
  
  return 0;
}
