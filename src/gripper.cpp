#include "mujoco_ros/gripper.h"



Gripper::Gripper(ros::NodeHandle& nh)
{
  driver_torque = 1.0; driver_min_pos = 0.05;
  
  // command subscriber
  grip_sub = nh.subscribe("command/gripper", 1, &Gripper::grip_cb, this);
}



bool Gripper::init(mjModel* m, mjData* d, ros::NodeHandle& nh,
  const std::vector<GripperFinger>& gf,
  const int grasp_eq_idx, const std::vector<int>& graspable_geomI)
{
  N_FINGERS = (int)gf.size();
  
  // get indexing, copy initial ftip poses (assuming open start)
  fingers = gf;
  for(int f=0; f<N_FINGERS; f++)
    for(int i=0; i<7; i++)
      fingers[f].defpose[i] = m->eq_data[7*fingers[f].lock_weldI+i];

  // start locked
  for(int f=0; f<N_FINGERS; f++) lock_default_pose(m,d, f);
  grip_cmd = gripper_state = false;
  grasped_bI = -1; nh.setParam("grasped_object", "");

  // graspable bodies
  grasp_weldI = grasp_eq_idx;
  for(size_t i=0; i<graspable_geomI.size(); i++)
    if(graspable_geomI[i] != -1)
    {
      graspable_gI.push_back(graspable_geomI[i]);
      graspable_bI.push_back(m->geom_bodyid[graspable_gI.back()]);
      std::string body_name_ = mj_id2name(m, mjOBJ_BODY, graspable_bI.back());
      graspable_bodies.push_back(body_name_);
    }
  
  return true;
}

void Gripper::proc(mjModel* m, mjData* d, ros::NodeHandle& nh,
  FreeBodyTracker* fb_tracker)
{
  if(gripper_state != grip_cmd)
  {
    // unlock ftip welds, set motor torquess
    double torque_;
    if(grip_cmd) torque_ = driver_torque; // close
    else torque_ = -driver_torque; // open
    
    for(int f=0; f<N_FINGERS; f++)
    {
      m->eq_active[fingers[f].lock_weldI] = false;
      d->ctrl[fingers[f].motorI] = torque_;
    }
    // update status flag 
    gripper_state = grip_cmd;
  }
  
  if(gripper_state) // close
  {
    if((grasped_bI == -1) && grasp_checks(m,d))
    {
      // lock ftips
      for(int f=0; f<N_FINGERS; f++)
        set_weld_relpose(m,d, fingers[f].lock_weldI);
      // set grasp weld + rosparam
      m->eq_obj2id[grasp_weldI] = graspable_bI[grasped_bI];
      set_weld_relpose(m,d, grasp_weldI, graspedTF);
      nh.setParam("grasped_object", graspable_bodies[grasped_bI]);
      
      // check available free-body tracking for grasped object
      fb_track = (fb_tracker->find_bI(graspable_bI[grasped_bI]) != -1);
    }
    
    // free body tracking for grasped object
    if((grasped_bI != -1) && fb_track)
    {
      tf::Transform w_g_tf_; xpose_to_tf(m,d, w_g_tf_, m->eq_obj1id[grasp_weldI]);
      tf::Transform w_go_tf_ = w_g_tf_ * graspedTF;
      fb_tracker->shift(m,d, graspable_bI[grasped_bI], w_go_tf_);
    }
  }
  else // open
  {
    if(grasped_bI != -1) // clear grasp
    {
      m->eq_active[grasp_weldI] = false; grasped_bI = -1;
      nh.setParam("grasped_object", "");
    }
    
    // lock fingertips if drivers moved far enough
    for(int f=0; f<N_FINGERS; f++)
      if(!m->eq_active[fingers[f].lock_weldI] &&
        (d->qpos[fingers[f].joint_posI] <= driver_min_pos))
      {
        d->ctrl[fingers[f].motorI] = 0.0;
        lock_default_pose(m,d, f);
      }
  }
}

    
    
//// ROS

void Gripper::grip_cb(const std_msgs::Bool& msg)
{
  grip_cmd = msg.data;
}



//// other

void Gripper::lock_default_pose(mjModel* m, mjData* d, const int fI)
{
  for(int i=0; i<7; i++)
    m->eq_data[7*fingers[fI].lock_weldI+i] = fingers[fI].defpose[i];
      
  m->eq_active[fingers[fI].lock_weldI] = true;
}

void Gripper::set_weld_relpose(mjModel* m, mjData* d, const int weldI)
{
  tf::Transform tfo_; set_weld_relpose(m,d, weldI, tfo_);
}

void Gripper::set_weld_relpose(mjModel* m, mjData* d, const int weldI, tf::Transform& tf_out)
{
  xpose_to_tf_rel(m,d, tf_out, m->eq_obj1id[weldI],m->eq_obj2id[weldI]);

  m->eq_data[7*weldI+0] = tf_out.getOrigin()[0];
  m->eq_data[7*weldI+1] = tf_out.getOrigin()[1];
  m->eq_data[7*weldI+2] = tf_out.getOrigin()[2];
  m->eq_data[7*weldI+3] = tf_out.getRotation()[3];
  m->eq_data[7*weldI+4] = tf_out.getRotation()[0];
  m->eq_data[7*weldI+5] = tf_out.getRotation()[1];
  m->eq_data[7*weldI+6] = tf_out.getRotation()[2];
  
  m->eq_active[weldI] = true;
}



bool Gripper::grasp_checks(mjModel* m, mjData* d)
{
  for(int i=0; i<(int)graspable_gI.size(); i++)
    if(grasp_check(m,d, graspable_gI[i]))
      { grasped_bI = i; return true; }
  
  return false;
}

bool Gripper::grasp_check(mjModel* m, mjData* d, const int target_gI)
{
  // check contact of each finger
  std::vector<bool> con_(N_FINGERS, false);  
  for(int c=0; c<d->ncon; c++)
    for(int f=0; f<N_FINGERS; f++)
      if(!con_[f])
      {
        bool fc1_ = (d->contact[c].geom1 == fingers[f].ftip_geomI),
          fc2_ = (d->contact[c].geom2 == fingers[f].ftip_geomI),
          tc1_ = (d->contact[c].geom1 == target_gI),
          tc2_ = (d->contact[c].geom2 == target_gI);
        con_[f] = ((fc1_ && tc2_) || (fc2_ && tc1_));
      }
  
  // true if all fingers have contact
  for(int f=0; f<N_FINGERS; f++)
    if(!con_[f]) return false;
    
  return true;
}
