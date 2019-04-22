#include "mujoco_ros/freebody_joint_tracking.h"



FreeBodyTracker::FreeBodyTracker(const double sim_period)
{
  dt = sim_period;
}



bool FreeBodyTracker::init(mjModel* m, mjData* d,
  const std::vector<std::string>& freebody_names)
{
  free_bodies.clear();

  for(int i=0; i<freebody_names.size(); i++)
  {
    int bI_ = mj_name2id(m, mjOBJ_BODY, freebody_names[i].c_str());
    if((bI_ != -1) && (m->body_jntnum[bI_] == 6))
    {
      FreeBody fb_;
      fb_.bI = bI_; fb_.pbI = m->body_parentid[fb_.bI];
      for(int j=0; j<6; j++)
      {
        fb_.jI[j].I = m->body_jntadr[bI_] + j; 
        fb_.jI[j].p = m->jnt_qposadr[fb_.jI[j].I];
        fb_.jI[j].v = m->jnt_dofadr[fb_.jI[j].I];
      }
      
      fb_.parent_tracker.bI = -1;
      fb_.parent_tracker.vel_tup.clear();
      
      free_bodies.push_back(fb_);
    }
  }
  
  N_FB = (int)free_bodies.size();
  
  return (N_FB != 0);
}

void FreeBodyTracker::proc(mjModel* m, mjData* d)
{
  for(int b=0; b<N_FB; b++)
    if(free_bodies[b].parent_tracker.bI != -1)
    {
      // update tracking data where set
      TrackingData& td_ = free_bodies[b].parent_tracker;
      rel_pose_as_tuple(m,d, td_.pos_tup, td_.bI, free_bodies[b].bI);
      
      if(td_.vel_tup.empty()) td_.vel_tup = std::vector<double>(6, 0.0);
      else
        for(int j=0; j<3; j++)
        {
          td_.vel_tup[j] = (td_.pos_tup[j] - td_.pos_tup_l[j]) / dt;
          td_.vel_tup[3+j] = wrap_pi_diff(td_.pos_tup[3+j] - td_.pos_tup_l[3+j]) / dt;
        }
      
      td_.pos_tup_l = td_.pos_tup;
    }
}



//// other

bool FreeBodyTracker::set_tracking_parent(const int bI, const int tpbI)
{
  int i_ = find_bI(bI);
  if(i_ == -1) return false;
  
  free_bodies[i_].parent_tracker.bI = tpbI;
  
  if(tpbI == -1) free_bodies[i_].parent_tracker.vel_tup.clear();
  
  return true;
}

bool FreeBodyTracker::get_tracking_data(const int bI,
  std::vector<double>& pos_tup, std::vector<double>& vel_tup)
{
  int i_ = find_bI(bI);
  if(i_ == -1) return false;
  
  pos_tup = free_bodies[i_].parent_tracker.pos_tup;
  vel_tup = free_bodies[i_].parent_tracker.vel_tup;
  
  return true;
}

bool FreeBodyTracker::shift(mjModel* m, mjData* d, const int bI, const tf::Transform& w_t_tf)
{
  int i_ = find_bI(bI);
  if(i_ == -1) return false;
  
  FreeBody& fb_ = free_bodies[i_];
  
  // parent -> child
  std::vector<double> p_c_tup_;
  rel_pose_as_tuple(m,d, p_c_tup_, fb_.pbI, bI);
  // parent -> target
  tf::Transform w_p_tf_; xpose_to_tf(m,d, w_p_tf_, fb_.pbI);
  tf::Transform p_t_tf_ = w_p_tf_.inverse() * w_t_tf;
  std::vector<double> p_t_tup_; transform_to_6tuple(p_t_tup_, p_t_tf_);
  
  // shift joints accordingly
  for(int j=0; j<3; j++)
  {
    d->qpos[fb_.jI[j].p] += p_t_tup_[j] - p_c_tup_[j]; // slide
    d->qpos[fb_.jI[3+j].p] += wrap_pi_diff(p_t_tup_[3+j] - p_c_tup_[3+j]); // hinge
    
    d->qvel[fb_.jI[j].v] = d->qvel[fb_.jI[3+j].v] = 0.0; // velocities
  }
  
  return true;
}

int FreeBodyTracker::find_bI(const int bI)
{
  for(int i=0; i<N_FB; i++)
    if(free_bodies[i].bI == bI) return i;
    
  return -1;
}
