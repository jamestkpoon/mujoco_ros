#ifndef THREADED_BODY_LOCKING_H
#define THREADED_BODY_LOCKING_H



#include "mujoco_ros/common.h"
#include "mujoco_ros/freebody_joint_tracking.h"

// ROS
#include "mujoco_ros/ThreadLock.h"



struct ThreadedConnection
{
  int bI; double pitch;
  std::vector<int> jord; JointIndex jI[6];
};



struct ThreadedBodyLocker
{
  
  public:
  
    ThreadedBodyLocker(ros::NodeHandle& nh,
      const double sim_period);
    ~ThreadedBodyLocker() {}
    
    bool init();
    void proc(mjModel* m, mjData* d,
      FreeBodyTracker* fb_tracker);
    
    
  
  private:

    // ROS
    bool tl_cb(mujoco_ros::ThreadLock::Request& req, mujoco_ros::ThreadLock::Response& res);
    bool handle_request(mjModel* m, mjData* d,
        FreeBodyTracker* fb_tracker, const mujoco_ros::ThreadLock::Request& req);
    
    ros::ServiceServer threadlock_srv;
    std::vector<mujoco_ros::ThreadLock::Request> srv_requests;
    
    // mujoco
    std::vector<ThreadedConnection> threaded_connections;
    double JNT_LOCK_TOL, dt;
  
};



#endif
