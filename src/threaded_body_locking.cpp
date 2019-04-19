#include "mujoco_ros/threaded_body_locking.h"



ThreadedBodyLocker::ThreadedBodyLocker(ros::NodeHandle& nh)
{
  JNT_LOCK_TOL = 0.001;
  
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
  // request queue
  for(int i=0; i<(int)srv_requests.size(); i++)
    handle_request(m,d, fb_tracker, srv_requests[i]); 
  srv_requests.clear();
  
  // handle connections
  
}



//// ROS

bool ThreadedBodyLocker::tl_cb(mujoco_ros::ThreadLock::Request& req,
  mujoco_ros::ThreadLock::Response& res)
{
  srv_requests.push_back(req);
  
  return true;
}

bool ThreadedBodyLocker::handle_request(mjModel* m, mjData* d,
  FreeBodyTracker* fb_tracker, const mujoco_ros::ThreadLock::Request& req)
{
  if(req.attach_flag)
  {
    // joint indexing, assumes 6 joints [ tx,ty,tz, rx,ry,rz ]
    int bI_ = mj_name2id(m, mjOBJ_BODY, req.fastener_name.c_str());
    if((bI_ == -1) || (m->body_jntnum[bI_] != 6)) return false;
    
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
    tc_.fastener_name = req.fastener_name; tc_.pitch = req.pitch;
    for(int i=0; i<6; i++)
    {
      tc_.jI[i].I = m->body_jntadr[bI_] + jI_ord_[i];
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
    
    // append connection, return OK
    threaded_connections.push_back(tc_);
    return true;
  }
  else
  {
    for(size_t i=0; i<threaded_connections.size(); i++)
      if(threaded_connections[i].fastener_name == req.fastener_name)
      {
        // free other joints
        for(int j=2; j<6; j++)
          m->jnt_limited[threaded_connections[i].jI[j].I] = false;
        // erase connection, return OK
        threaded_connections.erase(threaded_connections.begin()+i);
        return true;
      }
  }
}

//// 


//    res.ok = false; return true;
//  }
//}

//void MujocoNode::handle_threaded_connections()
//{
//  for(size_t i=0; i<threaded_connections.size(); i++)
//  {
//    d->qvel[threaded_connections[i].jI[0].v] = d->qvel[threaded_connections[i].jI[1].v] * threaded_connections[i].pitch;
//    d->qpos[threaded_connections[i].jI[0].p] += d->qvel[threaded_connections[i].jI[0].v] * FPS_period;
//  }
//}

