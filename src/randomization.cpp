#include "mujoco_ros/randomization.h"



Randomizer::Randomizer(ros::NodeHandle& nh)
{
  rng = new RNG((int)ros::Time::now().toSec());

  randtex_srv = nh.advertiseService("rand/textural", &Randomizer::randomize_textural_cb, this);
  randphys_srv = nh.advertiseService("rand/physical", &Randomizer::randomize_physical_cb, this);

  undotex_srv = nh.advertiseService("rand/undo/textural", &Randomizer::undo_tex_cb, this);
  undophys_srv = nh.advertiseService("rand/undo/physical", &Randomizer::undo_phys_cb, this);
}

Randomizer::~Randomizer()
{
  delete rng;
}



bool Randomizer::init()
{
  randtex_req.clear(); randphys_req.clear();

  childTF_shift.clear();
  childTF_shift_trigger.next = childTF_shift_trigger.now = false;
  childTF_undo_trigger.next = childTF_undo_trigger.now = false;
  
  tex_shifts.clear(); undo_tex_shifts = false;
  phys_shifts.clear(); undo_phys_shifts = false;
  
  return true;
}

void Randomizer::proc(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker)
{
  // textural randomizations
  if(!randtex_req.empty() && (!undo_tex_shifts && !tex_shifts.empty())) tex_shifts.clear();
  
  for(int i=0; i<(int)randtex_req.size(); i++)
    handle_request_tex(m,d, randtex_req[i]);
  randtex_req.clear();

  if(undo_tex_shifts)
    { undo_randtex(m,d); undo_tex_shifts = false; }
  
  // physical randomizations
  if(!randphys_req.empty() && (!undo_phys_shifts && !phys_shifts.empty()))
    { phys_shifts.clear(); childTF_shift.clear(); }
  
  for(int i=0; i<(int)randphys_req.size(); i++)
    handle_request_phys(m,d, fb_tracker, randphys_req[i]);
  randphys_req.clear();
  handle_child_repose_delay(m,d, fb_tracker, childTF_shift_trigger);
  
  if(undo_phys_shifts)
    { undo_randphys(m,d); undo_phys_shifts = false; }
  handle_child_repose_delay(m,d, fb_tracker, childTF_undo_trigger);  
}



//// ROS

bool Randomizer::randomize_textural_cb(mujoco_ros::RandomizeTexturalAttribute::Request& req, mujoco_ros::RandomizeTexturalAttribute::Response& res)
{
  randtex_req.push_back(req);
  
  return true;
}

bool Randomizer::randomize_physical_cb(mujoco_ros::RandomizePhysicalAttribute::Request& req, mujoco_ros::RandomizePhysicalAttribute::Response& res)
{
  randphys_req.push_back(req);
  
  return true;
}

std::vector<bool> Randomizer::handle_request_tex(mjModel* m, mjData* d,
  const mujoco_ros::RandomizeTexturalAttribute::Request& req)
{
  std::vector<bool> res_;

  if(req.noise.size() == 8*req.material_names.size()) 
    for(int i=0; i<(int)req.material_names.size(); i++)
    {
      int matI_ = mj_name2id(m, mjOBJ_MATERIAL, req.material_names[i].c_str());
      if(matI_ != -1)
      {
        // backup
        tex_shift tex_shift_; tex_shift_.matI = matI_;
        tex_shift_.essr[0] = m->mat_emission[matI_];
        tex_shift_.essr[1] = m->mat_specular[matI_];
        tex_shift_.essr[2] = m->mat_shininess[matI_];
        tex_shift_.essr[3] = m->mat_reflectance[matI_];
        for(int c=0; c<4; c++) tex_shift_.rgba[c] = m->mat_rgba[4*matI_+c];
          
        tex_shifts.push_back(tex_shift_);
        
        // req.noise: [ emission*1, specular*1, shininess*1, reflectance*1, rgba*4 ] * N_material_names
        m->mat_emission[matI_] = rng->rand_clip(m->mat_emission[matI_], req.noise[8*i+0], 0.0,1.0);
        m->mat_specular[matI_] = rng->rand_clip(m->mat_specular[matI_], req.noise[8*i+1], 0.0,1.0);
        m->mat_shininess[matI_] = rng->rand_clip(m->mat_shininess[matI_], req.noise[8*i+2], 0.0,1.0);
        m->mat_reflectance[matI_] = rng->rand_clip(m->mat_reflectance[matI_], req.noise[8*i+3], 0.0,1.0);
        for(int c=0; c<4; c++)
          m->mat_rgba[4*matI_+c] = rng->rand_clip(m->mat_rgba[4*matI_+c], req.noise[8*i+4+c], 0.0,1.0);
          
        res_.push_back(true);
      }
      else res_.push_back(false);
    }
    
  return res_;
}

std::vector<bool> Randomizer::handle_request_phys(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker, const mujoco_ros::RandomizePhysicalAttribute::Request& req)
{
  std::vector<bool> res_;
  
  // get body indices, check joint data lengths
  int nb_ = (int)req.body_names.size(), bI_[nb_], nj_[nb_], nj_cumsum_=0;
  std::vector<bool> jointed_(nb_, true);
  for(int i=0; i<nb_; i++)
  {
    bI_[i] = mj_name2id(m, mjOBJ_BODY, req.body_names[i].c_str());
    if(bI_[i] != -1)
    {
      nj_[i] = m->body_jntnum[bI_[i]];
      if(nj_[i] == 0) { nj_[i] = 6; jointed_[i] = false; }
      
      nj_cumsum_ += nj_[i];
    }
    else return res_;
  }
  
//  int cam_bI_ = mj_name2id(m, mjOBJ_BODY, "ros_cam_body");
//  ROS_INFO("%d %d", cam_bI_, m->body_jntnum[cam_bI_]);
//  m->body_pos[cam_bI_*3+0] += 1.0;
  
  // randomize if total lengths match
  if((nj_cumsum_ == (int)req.noise.size()) && (nb_ == (int)req.hold_6dof_children.size()))
  {
    int noiseI_ = 0;
    for(int i=0; i<nb_; i++)
    {
      // parent body
      phys_shift phys_shift_; phys_shift_.bI = bI_[i];
      phys_shift_.offsets.resize(nj_[i]);
      
      for(int j=0; j<nj_[i]; j++)
        phys_shift_.offsets[j] = req.noise[noiseI_++] * rng->rand_pm1();
      
      if(jointed_[i]) // adjust joints
      {
        phys_shift_.jI.resize(nj_[i]);
        
        for(int j=0; j<nj_[i]; j++)
        {
          JointIndex& jI_ = phys_shift_.jI[j]; jI_.I = m->body_jntadr[phys_shift_.bI];
          jI_.p = m->jnt_qposadr[jI_.I+j]; jI_.v = m->jnt_dofadr[jI_.I+j];

          d->qpos[jI_.p] += phys_shift_.offsets[j];
          d->qvel[jI_.v] = 0.0;
        }
      }
      else // adjust jointless body
      {
        phys_shift_.jI.clear();
        offset_disjointed_body(m,d, phys_shift_);
      }
      
      phys_shifts.push_back(phys_shift_);
      res_.push_back(true);

      // children
      if(req.hold_6dof_children[i])
        for(int b=0; b<m->nbody; b++)
          if(childOK(m,d, fb_tracker, b,phys_shift_.bI))
          {
            ChildTF pc_;
            pc_.bI = b; // child body index
            xpose_to_tf(m,d, pc_.pose, b); // child pose wrt world
            childTF_shift.push_back(pc_);
          }
    }
    
    childTF_shift_trigger.next = !childTF_shift.empty();
  }
  
  return res_;
}



bool Randomizer::undo_tex_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  undo_tex_shifts = !tex_shifts.empty();
  
  return true;
}

bool Randomizer::undo_phys_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  undo_phys_shifts = !phys_shifts.empty();
  
  return true;
}

void Randomizer::undo_randtex(mjModel* m, mjData* d)
{
  for(int i=(int)tex_shifts.size()-1; i>-1; i--) // go backwards
  {
    tex_shift& ts_ = tex_shifts[i];
    m->mat_emission[ts_.matI] = ts_.essr[0];
    m->mat_specular[ts_.matI] = ts_.essr[1];
    m->mat_shininess[ts_.matI] = ts_.essr[2];
    m->mat_reflectance[ts_.matI] = ts_.essr[3];
    for(int c=0; c<4; c++) m->mat_rgba[4*ts_.matI+c] = ts_.rgba[c];      
  }
  
  tex_shifts.clear();
}

void Randomizer::undo_randphys(mjModel* m, mjData* d)
{
  for(int i=(int)phys_shifts.size()-1; i>-1; i--)
  {
    phys_shift& ps_ = phys_shifts[i];
    
    if(!ps_.jI.empty())
      for(int j=0; j<(int)ps_.jI.size(); j++)
      {
        d->qpos[ps_.jI[j].p] -= ps_.offsets[j];
        d->qvel[ps_.jI[j].v] = 0.0;
      }
    else
    {
      for(int j=0; j<6; j++) ps_.offsets[j] = -ps_.offsets[j];
      offset_disjointed_body(m,d, ps_);
    }
  }
  phys_shifts.clear();

  childTF_undo_trigger.next = !childTF_shift.empty();
}



//// other

void Randomizer::offset_disjointed_body(mjModel* m, mjData* d,
  const phys_shift& shift)
{
  geometry_msgs::Quaternion quat_msg_;
  quat_msg_.x = m->body_quat[shift.bI*4+1];
  quat_msg_.y = m->body_quat[shift.bI*4+2];
  quat_msg_.z = m->body_quat[shift.bI*4+3];
  quat_msg_.w = m->body_quat[shift.bI*4+0];
  tf::Quaternion quat_; tf::quaternionMsgToTF(quat_msg_, quat_);
  double eul_[3]; tfQuat_to_eul(quat_, eul_[0],eul_[1],eul_[2]);
  
  for(int j=0; j<3; j++)
  {
    m->body_pos[3*shift.bI+j] += shift.offsets[j];
    eul_[j] += shift.offsets[3+j];
  }
  
  quat_msg_ = tf::createQuaternionMsgFromRollPitchYaw(eul_[0],eul_[1],eul_[2]);
  m->body_quat[shift.bI*4+1] = quat_msg_.x;
  m->body_quat[shift.bI*4+2] = quat_msg_.y;
  m->body_quat[shift.bI*4+3] = quat_msg_.z;
  m->body_quat[shift.bI*4+0] = quat_msg_.w;
}

bool Randomizer::childOK(mjModel* m, mjData* d, FreeBodyTracker* fb_tracker,
  const int cI, const int pI)
{
  if((cI == pI) || (fb_tracker->find_bI(cI) == -1)) return false;
  
  bool ancestry_ = false; int pI_ = cI;
  while(pI_ != 0)
  {
    // check if parent structure already stored
    for(int i=0; i<(int)childTF_shift.size(); i++)
      if(childTF_shift[i].bI == pI_) return false;
    
    // check if parent reached
    if(!ancestry_ && (pI_ == pI)) ancestry_ = true;
      
    pI_ = m->body_parentid[pI_];
  }
  
  return ancestry_;
}

void Randomizer::handle_child_repose_delay(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker, delay_trigger& trigger)
{
  if(trigger.next)
  {
    if(trigger.now)
    {
      for(int i=0; i<(int)childTF_shift.size(); i++)
        fb_tracker->shift(m,d, childTF_shift[i].bI, childTF_shift[i].pose);
      trigger.next = trigger.now = false;
    }
    else trigger.now = true;
  }
}
