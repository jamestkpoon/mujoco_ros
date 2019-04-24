#ifndef RANDOMIZATION_H
#define RANDOMIZATION_H



#include "mujoco_ros/common.h"
#include "mujoco_ros/freebody_joint_tracking.h"

// ROS
#include "mujoco_ros/RandomizeTexturalAttribute.h"
#include "mujoco_ros/RandomizePhysicalAttribute.h"
#include "std_srvs/Empty.h"

// RNG
#include "boost/random.hpp"
#include <boost/format.hpp>



struct ChildTF
{
  int bI; tf::Transform pose;
};

struct delay_trigger
{
  bool next, now;
};

struct tex_shift
{
  int matI; double essr[4], rgba[4];
};

struct phys_shift
{
  std::vector<JointIndex> jI;
  std::vector<double> offsets;
};



class RNG
{

  public:
    RNG(const int seed)
    {
      rng = boost::mt19937(seed);
    }
    ~RNG() {}
    
    double rand_01()
    {
      static boost::uniform_01<boost::mt19937> zeroone(rng);
      return zeroone();
    }

    double rand_pm1()
    {
      return 2*rand_01() - 1.0;
    }

    double rand_clip(const double mean, const double noise, const double lb, const double ub)
    {
      if(mean == lb) return lb + (noise * rand_01());
      else if(mean == ub) return ub - (noise * rand_01());
      else return std::max(lb, std::min(mean + (noise * rand_pm1()), ub));
    }
  
  
  
  private:
  
    boost::random::mt19937 rng;

};



class Randomizer
{

  public:
  
    Randomizer(ros::NodeHandle& nh);
    ~Randomizer();
    
    bool init();
    void proc(mjModel* m, mjData* d,
      FreeBodyTracker* fb_tracker);
      
      
  
  private:

    // ROS
    bool randomize_textural_cb(mujoco_ros::RandomizeTexturalAttribute::Request& req, mujoco_ros::RandomizeTexturalAttribute::Response& res);
    bool randomize_physical_cb(mujoco_ros::RandomizePhysicalAttribute::Request& req, mujoco_ros::RandomizePhysicalAttribute::Response& res);
    std::vector<bool> handle_request_tex(mjModel* m, mjData* d, const mujoco_ros::RandomizeTexturalAttribute::Request& req);
    std::vector<bool> handle_request_phys(mjModel* m, mjData* d, const mujoco_ros::RandomizePhysicalAttribute::Request& req);

    ros::ServiceServer randtex_srv, randphys_srv;
    std::vector<mujoco_ros::RandomizeTexturalAttribute::Request> randtex_req;
    std::vector<mujoco_ros::RandomizePhysicalAttribute::Request> randphys_req;
    
    bool undo_tex_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool undo_phys_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    
    ros::ServiceServer undotex_srv, undophys_srv;
    bool undo_tex_shifts, undo_phys_shifts;
    
    // mujoco
    bool childOK(mjModel* m, mjData* d, const int cI, const int pI);
    void handle_child_repose_delay(mjModel* m, mjData* d,
      FreeBodyTracker* fb_tracker, delay_trigger& trigger);
    void undo_randtex(mjModel* m, mjData* d);
    void undo_randphys(mjModel* m, mjData* d);
    
    std::vector<ChildTF> childTF_shift; delay_trigger childTF_shift_trigger, childTF_undo_trigger;
    std::vector<tex_shift> tex_shifts; std::vector<phys_shift> phys_shifts;
    
    // RNG
    RNG* rng;
    
};



#endif
