#include "mujoco_ros/common.h"



void tfQuat_to_eul(const tf::Quaternion& q, double& r, double& p, double& y)
{
  tf::Matrix3x3 m_(q); m_.getRPY(r,p,y);
}



void xpose_to_tf(mjModel* m, mjData* d, tf::Transform& tf_out, const int bI)
{
  tf_out.setOrigin(tf::Vector3(d->xpos[3*bI+0], d->xpos[3*bI+1], d->xpos[3*bI+2]));
  tf_out.setRotation(tf::Quaternion(d->xquat[4*bI+1], d->xquat[4*bI+2], d->xquat[4*bI+3], d->xquat[4*bI+0]));
}

void xpose_to_tf_rel(mjModel* m, mjData* d, tf::Transform& tf_out, const int pbI, const int cbI)
{
  tf::Transform pb_tf_; xpose_to_tf(m,d, pb_tf_, pbI);
  tf::Transform cb_tf_; xpose_to_tf(m,d, cb_tf_, cbI);
  
  tf_out = pb_tf_.inverse() * cb_tf_;
}

void transform_to_6tuple(std::vector<double>& tup, const tf::Transform& pose_tf)
{
  double eul_[3]; tfQuat_to_eul(pose_tf.getRotation(), eul_[0],eul_[1],eul_[2]);
  
  tup.resize(6); // [ tx,ty,tz, rx,ry,rz ]
  for(int i=0; i<3; i++)
  {
    tup[i] = pose_tf.getOrigin()[i];
    tup[3+i] = eul_[i];
  }
}

void rel_pose_as_tuple(mjModel* m, mjData* d, std::vector<double>& tup, const int pI, const int cI)
{
  tf::Transform pc_tf_; xpose_to_tf_rel(m,d, pc_tf_, pI,cI);
  transform_to_6tuple(tup, pc_tf_);
}
