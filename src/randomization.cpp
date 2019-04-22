#include "mujoco_ros/randomization.h"



Randomizer::Randomizer(ros::NodeHandle& nh)
{
  randtex_srv = nh.advertiseService("rand/textural", &Randomizer::randomize_textural_cb, this);
  randphys_srv = nh.advertiseService("rand/physical", &Randomizer::randomize_physical_cb, this);
}



bool Randomizer::init()
{
  randtex_req.clear(); randphys_req.clear();
  phys_child_pose_fix.clear();
  
  parent_fbI.clear(); child_fbI.clear();
  parent_fbI_trigger.next = parent_fbI_trigger.now = false;
  child_fbI_trigger.next = child_fbI_trigger.now = false;
  
  return true;
}

void Randomizer::proc(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker)
{
  // textural randomization requests
  for(int i=0; i<(int)randtex_req.size(); i++)
    handle_request_tex(m,d, randtex_req[i]);
  randtex_req.clear();
  
  // physical randomization requests
  for(int i=0; i<(int)randphys_req.size(); i++)
    handle_request_phys(m,d, fb_tracker, randphys_req[i]);
  randphys_req.clear();
  
  // t-1 hold-overs
  if(parent_fbI_trigger.next)
  {
    if(parent_fbI_trigger.now)
    {
      handle_phys_next_step(m,d, fb_tracker);
      parent_fbI_trigger.next = parent_fbI_trigger.now = false;
    }
    else parent_fbI_trigger.now = true;
  }
  
  if(child_fbI_trigger.next)
  {
    if(child_fbI_trigger.now)
    {
      handle_phys_c_next_step(fb_tracker);
      child_fbI_trigger.next = child_fbI_trigger.now = false;
    }
    else child_fbI_trigger.now = true;
  }
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

std::vector<bool> Randomizer::handle_request_tex(mjModel* m, mjData* d, const mujoco_ros::RandomizeTexturalAttribute::Request& req)
{
  std::vector<bool> res_;

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
  int nb_ = (int)req.body_names.size(), bI_[nb_], nj_cumsum_=0;
  for(int i=0; i<nb_; i++)
  {
    bI_[i] = mj_name2id(m, mjOBJ_BODY, req.body_names[i].c_str());
    if(bI_[i] != -1) nj_cumsum_ += m->body_jntnum[bI_[i]];
    else return res_;
  }
  
  // randomize if total lengths match
  if((nj_cumsum_ == (int)req.noise.size()) && (nb_ == (int)req.hold_6dof_children.size()))
  {
    int noiseI_ = 0;
    for(int i=0; i<nb_; i++)
      if(m->body_jntnum[bI_[i]] == 0) res_.push_back(false);
      else
      {
        // parent body
        std::vector<bool> track_flags_(6, true);
        for(int j=0; j<m->body_jntnum[bI_[i]]; j++) track_flags_[j] = false;
        fb_tracker->set_track_flags(bI_[i], track_flags_);
        parent_fbI.push_back(bI_[i]);
        
        JointIndex jI_; jI_.I = m->body_jntadr[bI_[i]];
        for(int j=0; j<m->body_jntnum[bI_[i]]; j++)
        {
          jI_.p = m->jnt_qposadr[jI_.I+j]; jI_.v = m->jnt_dofadr[jI_.I+j];
          d->qpos[jI_.p] += req.noise[noiseI_++] * rand_pm1();
          d->qvel[jI_.v] = 0.0;
        }
        
        res_.push_back(true);

        // children, assumes 6 joints [ tx,ty,tz, rx,ry,rz ]
        for(int p=0; p<nb_; p++)
          if(req.hold_6dof_children[p])
          {
            for(int b=0; b<m->nbody; b++)
              if(childOK(m,d, b,bI_[p]))
              {
                PCtf pc_;
                pc_.p_bI = m->body_parentid[b]; // parent body index
                xpose_to_tf(m,d, pc_.world_c, b); // child pose wrt world
                for(int j=0; j<6; j++) // child joint indices
                {
                  pc_.jpI[j] = m->jnt_qposadr[m->body_jntadr[b]+j];
                  pc_.jvI[j] = m->jnt_dofadr[m->body_jntadr[b]+j];
                }
                
                phys_child_pose_fix.push_back(pc_);
                child_fbI.push_back(b);
              }
          }
      }

    parent_fbI_trigger.next = !parent_fbI.empty();
  }
  
  return res_;
}



//// other

bool Randomizer::childOK(mjModel* m, mjData* d, const int cI, const int pI)
{
  bool lineage_ = false; int pI_ = cI;
  while(pI_ != 0)
  {
    pI_ = m->body_parentid[pI_];
    if(pI_ == pI) { lineage_ = true; break; }
  }
  
  return (lineage_ && (m->body_jntnum[cI] == 6));
}

void Randomizer::handle_phys_next_step(mjModel* m, mjData* d, FreeBodyTracker* fb_tracker)
{
  // children pose fixing
  child_fbI_trigger.next = !child_fbI.empty();
  fill_fbtrack_flags(fb_tracker, child_fbI, false);

  for(int i=0; i<(int)phys_child_pose_fix.size(); i++)
    fix_child_pose(m,d, phys_child_pose_fix[i]);
  phys_child_pose_fix.clear();
  
  // reset joint tracking flags (parents)
  fill_fbtrack_flags(fb_tracker, parent_fbI, true);
  parent_fbI.clear();
}

void Randomizer::handle_phys_c_next_step(FreeBodyTracker* fb_tracker)
{
  // reset joint tracking flags (children)
  fill_fbtrack_flags(fb_tracker, child_fbI, true);
  child_fbI.clear();
}

void Randomizer::fix_child_pose(mjModel* m, mjData* d, PCtf& pc)
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
  for(int i=0; i<6; i++) d->qvel[pc.jvI[i]] = 0.0;
}

void Randomizer::fill_fbtrack_flags(FreeBodyTracker* fb_tracker, std::vector<int>& bI, const bool flag)
{
  std::vector<bool> track_flags_(6, flag);
  for(int i=0; i<(int)bI.size(); i++)
    fb_tracker->set_track_flags(bI[i], track_flags_);
}
