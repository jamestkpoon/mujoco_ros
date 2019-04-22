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
      free_bodies.push_back(fb_);
//      xpose_to_tf(m,d, fb_.defpose, bI_); // world pose
//      fb_.track = std::vector<bool>(6, true);
//      fb_.zvels = std::vector<bool>(6, true);
      

    }
  }
  
  N_FB = (int)free_bodies.size();
  
  return (N_FB != 0);
}

void FreeBodyTracker::proc(mjModel* m, mjData* d)
{
//  for(int b=0; b<N_FB; b++)
//  {
    // new parent pose -> starting child pose
//    tf::Transform w_p_tf_; xpose_to_tf(m,d, w_p_tf_, free_bodies[b].pbI);
//    tf::Transform p_dc_tf_ = w_p_tf_.inverse() * free_bodies[b].defpose;
//    std::vector<double> p_dc_tup_; transform_to_6tuple(p_dc_tup_, p_dc_tf_);
//    // new parent pose -> new child pose
//    std::vector<double> p_nc_tup_;
//    rel_pose_as_tuple(m,d, p_nc_tup_, free_bodies[b].pbI,free_bodies[b].bI);
//    
//    // local differential tuple for joint velocities
//    std::vector<double> diff_(6, 0.0);
//    for(int i=0; i<6; i++)
//      if(free_bodies[b].zvels[i]) free_bodies[b].zvels[i] = false;
//      else diff_[i] = p_nc_tup_[i] - free_bodies[b].last_pc_tup[i];

//    free_bodies[b].last_pc_tup = p_nc_tup_;
//    
//    // set slide joint properties
//    for(int i=0; i<3; i++)
//      if(free_bodies[b].track[i])
//      {
//        d->qpos[free_bodies[b].jI[i].p] = p_nc_tup_[i] - p_dc_tup_[i];
//        d->qvel[free_bodies[b].jI[i].v] = diff_[i] * dt;
//      }

//    if(std::string(mj_id2name(m, mjOBJ_BODY, free_bodies[b].bI)) == "nut")
//    {
//      if(!free_bodies[b].track[0])
//      {
//        ROS_INFO("blooorg");
//        ROS_INFO("%f,%f,%f", d->qpos[free_bodies[b].jI[0].p],d->qpos[free_bodies[b].jI[1].p],d->qpos[free_bodies[b].jI[2].p]);
//        for(int i=0; i<6; i++)
//        {
//          ROS_INFO("%f, %f", p_dc_tup_[i], p_nc_tup_[i]);
//        }
//       }
//    }
//    
//    // set hinge joint properties
//    for(int i=3; i<6; i++)
//      if(free_bodies[b].track[i])
//      {
//        d->qpos[free_bodies[b].jI[i].p] = p_nc_tup_[i] - p_dc_tup_[i];
//        d->qvel[free_bodies[b].jI[i].v] = wrap_pi_diff(diff_[i]) * dt;
//      }
//  }
}



bool FreeBodyTracker::set_track_flags(const int bI, const std::vector<bool>& track)
{
//  for(int i=0; i<N_FB; i++)
//    if((free_bodies[i].bI == bI) && (track.size() == 6))
//    {
//      for(int j=0; j<6; j++)
//        free_bodies[i].zvels[j] = (!free_bodies[i].zvels[j] && track[j]);
//      
//      free_bodies[i].track = track;
//      
//      return true;
//    }
//     
//  return false;  
}

bool FreeBodyTracker::shift(mjModel* m, mjData* d, const int bI, const tf::Transform& w_t_tf)
{
   for(int b=0; b<N_FB; b++)
    if(free_bodies[b].bI == bI)
    {
      // parent -> child
      std::vector<double> p_c_tup_;
      rel_pose_as_tuple(m,d, p_c_tup_, free_bodies[b].pbI, bI);
      // parent -> target
      tf::Transform w_p_tf_; xpose_to_tf(m,d, w_p_tf_, free_bodies[b].pbI);
      tf::Transform p_t_tf_ = w_p_tf_.inverse() * w_t_tf;
      std::vector<double> p_t_tup_; transform_to_6tuple(p_t_tup_, p_t_tf_);
      
      // shift corresponding joints
      for(int j=0; j<3; j++)
      {
        d->qpos[free_bodies[b].jI[j].p] += p_t_tup_[j] - p_c_tup_[j]; // slide
        d->qpos[free_bodies[b].jI[3+j].p] += wrap_pi_diff(p_t_tup_[3+j] - p_c_tup_[3+j]); // hinge
        
        d->qvel[free_bodies[b].jI[j].v] = free_bodies[b].jI[3+j].v = 0.0; // velocities
      }
      
      return true;
    }
    
    return false; 
}
