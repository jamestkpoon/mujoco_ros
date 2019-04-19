#ifndef COMMON_H
#define COMMON_H



// MuJoCo
#include "mujoco.h"

// ROS
#include "ros/ros.h"
#include "tf/transform_datatypes.h"



// objects

struct JointIndex
{
  int I, p,v; // id,  position/velocity indices
};



// helper fns

void xpose_to_tf(mjModel* m, mjData* d, tf::Transform& tf_out, const int bI);
void xpose_to_tf_rel(mjModel* m, mjData* d, tf::Transform& tf_out, const int pbI, const int cbI);
void transform_to_6tuple(std::vector<double>& tup, const tf::Transform& pose_tf);
void rel_pose_as_tuple(mjModel* m, mjData* d, std::vector<double>& tup, const int pI, const int cI);

double wrap_pi_diff(const double x);



#endif
