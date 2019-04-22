#include "mujoco_ros/randomization.h"



Randomizer::Randomizer(ros::NodeHandle& nh)
{
  randtex_srv = nh.advertiseService("rand/textural", &Randomizer::randomize_textural_cb, this);
  randphys_srv = nh.advertiseService("rand/physical", &Randomizer::randomize_physical_cb, this);
}



bool Randomizer::init()
{
  randtex_req.clear(); randphys_req.clear();
  childTF_shift.clear();
  
  childTF_trigger.next = childTF_trigger.now = false;
  
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
    handle_request_phys(m,d, randphys_req[i]);
  randphys_req.clear();
  
  // from previous sim step
  if(childTF_trigger.next)
  {
    if(childTF_trigger.now)
    {
      restore_child_poses(m,d, fb_tracker);
      childTF_trigger.next = childTF_trigger.now = false;
    }
    else childTF_trigger.now = true;
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
  const mujoco_ros::RandomizePhysicalAttribute::Request& req)
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
            for(int b=0; b<m->nbody; b++)
              if(childOK(m,d, b,bI_[p]))
              {
                ChildTF pc_;
                pc_.bI = b; // child body index
                xpose_to_tf(m,d, pc_.pose, b); // child pose wrt world
                childTF_shift.push_back(pc_);
              }
      }
      
    childTF_trigger.next = !childTF_shift.empty();
  }
  
  return res_;
}



//// other

bool Randomizer::childOK(mjModel* m, mjData* d, const int cI, const int pI)
{
  if((cI == pI) || (m->body_jntnum[cI] != 6)) return false;
  
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

void Randomizer::restore_child_poses(mjModel* m, mjData* d, FreeBodyTracker* fb_tracker)
{
  // children pose fixing
  for(int i=0; i<(int)childTF_shift.size(); i++)
    fb_tracker->shift(m,d, childTF_shift[i].bI, childTF_shift[i].pose);
  childTF_shift.clear();
}
