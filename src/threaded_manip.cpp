#include <mujoco_ros/mujoco_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujocoUR5_thread_manip");
  MujocoUR5 ur5_;
  
  
  int r_ = mu5_.run();
  
  return 0;
}
