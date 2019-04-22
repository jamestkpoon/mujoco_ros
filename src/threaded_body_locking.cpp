#include "mujoco_ros/threaded_body_locking.h"



ThreadedBodyLocker::ThreadedBodyLocker(ros::NodeHandle& nh,
  const double sim_period)
{
  JNT_LOCK_TOL = 0.001;
  dt = sim_period;
  
  threadlock_srv = nh.advertiseService("thread_lock", &ThreadedBodyLocker::tl_cb, this);
}



bool ThreadedBodyLocker::init()
{
  srv_requests.clear(); threaded_connections.clear();
  
  return true;
}

void ThreadedBodyLocker::proc(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker)
{
  // process request queue
  for(int i=0; i<(int)srv_requests.size(); i++)
    handle_request(m,d, fb_tracker, srv_requests[i]); 
  srv_requests.clear();
  
  // handle connections
  for(int i=0; i<(int)threaded_connections.size(); i++)
  {
    d->qvel[threaded_connections[i].jI[0].v] = d->qvel[threaded_connections[i].jI[1].v] * threaded_connections[i].pitch;
    d->qpos[threaded_connections[i].jI[0].p] += d->qvel[threaded_connections[i].jI[0].v] * dt;
  }
}



//// ROS

bool ThreadedBodyLocker::tl_cb(mujoco_ros::ThreadLock::Request& req, mujoco_ros::ThreadLock::Response& res)
{
  srv_requests.push_back(req);
  
  return true;
}

bool ThreadedBodyLocker::handle_request(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker, const mujoco_ros::ThreadLock::Request& req)
{
  int req_bI_ = mj_name2id(m, mjOBJ_BODY, req.fastener_name.c_str());
  if((req_bI_ == -1) || (m->body_jntnum[req_bI_] != 6)) return false;

  if(req.attach_flag)
  {
    // joint indexing, assumes 6 joints [ tx,ty,tz, rx,ry,rz ]   
    std::vector<int> jI_ord_; 
    if(req.axis == "x")
    {
      int ord_[6] = { 0,3, 1,4, 2,5 };
      jI_ord_ = std::vector<int>(ord_,ord_+6);
    }
    else if(req.axis == "y")
    {
      int ord_[6] = { 1,4, 0,3, 2,5 };
      jI_ord_ = std::vector<int>(ord_,ord_+6);
    }
    else if(req.axis == "z")
    {
      int ord_[6] = { 2,5, 0,3, 1,4 };
      jI_ord_ = std::vector<int>(ord_,ord_+6);
    }
    else return false;
    
    ThreadedConnection tc_;
    tc_.bI = req_bI_; tc_.pitch = req.pitch;
    for(int i=0; i<6; i++)
    {
      tc_.jI[i].I = m->body_jntadr[tc_.bI] + jI_ord_[i];
      tc_.jI[i].p = m->jnt_qposadr[tc_.jI[i].I];
      tc_.jI[i].v = m->jnt_dofadr[tc_.jI[i].I];
    }
    
    // constrain other joints
    for(int i=2; i<6; i++)
    {
      m->jnt_range[2*tc_.jI[i].I+0] = d->qpos[tc_.jI[i].p] - JNT_LOCK_TOL;
      m->jnt_range[2*tc_.jI[i].I+1] = d->qpos[tc_.jI[i].p] + JNT_LOCK_TOL;
      m->jnt_limited[tc_.jI[i].I] = true; d->qvel[tc_.jI[i].v] = 0.0;
    }
    
    // append connection
    threaded_connections.push_back(tc_);
    
    // joint tracking flags
    std::vector<bool> track_flags_(6, true);
    track_flags_[jI_ord_[0]] = false; // stop tracking thread translation axis
    fb_tracker->set_track_flags(tc_.bI, track_flags_);
    
    return true;
  }
  else
  {
    for(size_t i=0; i<threaded_connections.size(); i++)
      if(threaded_connections[i].bI == req_bI_)
      {
        // free other joints
        for(int j=2; j<6; j++)
          m->jnt_limited[threaded_connections[i].jI[j].I] = false;
        // delete connection
        threaded_connections.erase(threaded_connections.begin()+i);
        
        // reset joint tracking flags
        fb_tracker->set_track_flags(req_bI_, std::vector<bool>(6,true));
        
        return true;
      }
      
    return false;
  }
}
