#include "mujoco_ros/freebody_joint_tracking.h"



FreeBodyTracker::FreeBodyTracker(const double sim_period)
{
  dt = sim_period;
}



bool FreeBodyTracker::init(mjModel* m, mjData* d,
  const std::vector<std::string>& freebody_names)
{
  for(int i=0; i<freebody_names.size(); i++)
  {
    int bI_ = mj_name2id(m, mjOBJ_BODY, freebody_names[i].c_str());
    if((bI_ != -1) && (m->body_jntnum[bI_] == 6))
    {
      FreeBody fb_;

      // indexing
      fb_.bI = bI_; fb_.pbI = m->body_parentid[fb_.bI];
      for(int j=0; j<6; j++)
      {
        fb_.jI[j].I = m->body_jntadr[bI_] + j; 
        fb_.jI[j].p = m->jnt_qposadr[fb_.jI[j].I];
        fb_.jI[j].v = m->jnt_dofadr[fb_.jI[j].I];
      }
      fb_.track = std::vector<bool>(6, true);
      
      xpose_to_tf(m,d, fb_.defpose, bI_); // world pose
      rel_pose_as_tuple(m,d, fb_.last_pc_tup, fb_.pbI,fb_.bI); // local pose
      
      free_bodies.push_back(fb_);
    }
  }
  
  N_FB = (int)free_bodies.size();
  
  return (N_FB != 0);
}

void FreeBodyTracker::proc(mjModel* m, mjData* d)
{
  for(int b=0; b<N_FB; b++)
  {
    // starting child pose wrt new parent pose
    tf::Transform w_p_tf_; xpose_to_tf(m,d, w_p_tf_, free_bodies[b].pbI);
    tf::Transform p_dc_tf_ = w_p_tf_.inverse() * free_bodies[b].defpose;
    std::vector<double> p_dc_tup_; transform_to_6tuple(p_dc_tup_, p_dc_tf_);
    // new child pose wrt new parent pose
    std::vector<double> p_nc_tup_;
    rel_pose_as_tuple(m,d, p_nc_tup_, free_bodies[b].pbI,free_bodies[b].bI);
    
    // joint differential tuple for velocities
    double diff_[6];
    for(int i=0; i<6; i++)
      diff_[i] = p_nc_tup_[i] - free_bodies[b].last_pc_tup[i];
    free_bodies[b].last_pc_tup = p_nc_tup_;
    
    // set slide joint properties
    for(int i=0; i<3; i++)
      if(free_bodies[b].track[i])
      {
        d->qpos[free_bodies[b].jI[i].p] = p_nc_tup_[i] - p_dc_tup_[i];
        d->qvel[free_bodies[b].jI[i].v] = diff_[i] * dt;
      }
    
    // set hinge joint properties
    for(int i=3; i<6; i++)
      if(free_bodies[b].track[i])
      {
        d->qpos[free_bodies[b].jI[i].p] = p_nc_tup_[i] - p_dc_tup_[i];
        d->qvel[free_bodies[b].jI[i].v] = wrap_pi_diff(diff_[i]) * dt;
      }
  }
}



bool FreeBodyTracker::set_track_flags(const int bI, const std::vector<bool>& track)
{
  if(track.size() != 6) return false;
  
  for(int i=0; i<N_FB; i++)
    if(free_bodies[i].bI == bI)
      { free_bodies[i].track = track; return true; }
    
  return false;  
}
