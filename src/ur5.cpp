#include "mujoco_ros/ur5.h"



UR5::UR5(ros::NodeHandle& nh, const double reflexxes_dt)
{
  // properties from v-rep
  maxVel =  180 * (M_PI/180); maxAccel = 40 * (M_PI/180); maxJerk = 80 * (M_PI/180);
  DOF = 6;
  
  // joint position command subscriber
  jpos_sub = nh.subscribe("command/joint_positions", 1, &UR5::jpos_cb, this);
  // joint state publisher
  jstate_pub = nh.advertise<std_msgs::Float32MultiArray>("sensors/joint_state", 1);

  // initialize Reflexxes
  rml_api = new ReflexxesAPI(DOF, reflexxes_dt);
  rml_i = new RMLPositionInputParameters(DOF);
  rml_o = new RMLPositionOutputParameters(DOF);
  rml_flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;
}

UR5::~UR5()
{
  // free Reflexxes resources
  delete rml_api; delete rml_i; delete rml_o;
}



bool UR5::init(mjModel* m, mjData* d, ros::NodeHandle& nh,
  const std::vector<std::string>& joint_names)
{
  // indexing
  if(joint_names.size() != DOF) return false;

  jI.resize(DOF);
  for(int i=0; i<DOF; i++)
  {
    int i_ = mj_name2id(m, mjOBJ_JOINT, joint_names[i].c_str());
    jI[i].p = m->jnt_qposadr[i_]; jI[i].v = m->jnt_dofadr[i_];
  }

  // move to an initial position
  traj_in = false; traj_started = false;
  std_msgs::Float32MultiArray initial_joint_pose_;
  initial_joint_pose_.data = std::vector<float>(DOF, 0.0);
  initial_joint_pose_.data[1] = -M_PI/2;
  for(int i=0; i<DOF; i++)
  {
    d->qpos[jI[i].p] = initial_joint_pose_.data[i];
    d->qvel[jI[i].v] = 0.0;
  }
  jpos_cb(initial_joint_pose_);
  
  // Reflexxes init
  for(int i=0; i<DOF; i++)
  {
    rml_i->CurrentPositionVector->VecData[i] = d->qpos[jI[i].p];
    rml_i->CurrentVelocityVector->VecData[i] = d->qvel[jI[i].v];
    rml_i->CurrentAccelerationVector->VecData[i] = 0.0;
            
    rml_i->MaxVelocityVector->VecData[i] = maxVel;
    rml_i->MaxAccelerationVector->VecData[i] = maxAccel;
    rml_i->MaxJerkVector->VecData[i] = maxJerk;
    
    rml_i->SelectionVector->VecData[i] = true;
  }
  
  nh.setParam("moving", false);

  return true;
}

void UR5::proc(mjModel* m, mjData* d, ros::NodeHandle& nh)
{
  if(traj_in)
  {
    // start new motion if not already
    if(!traj_started)
    {
      for(int i=0; i<DOF; i++)
      {
        // set target position and velocity
        rml_i->TargetPositionVector->VecData[i] = jpos[traj_step][i];
        rml_i->TargetVelocityVector->VecData[i] = jvel[traj_step][i];
      }
      nh.setParam("moving", true); traj_started = true;
    }
    
    // continue motion
    int rml_result_ = rml_api->RMLPosition(*rml_i,rml_o, rml_flags);
    *rml_i->CurrentPositionVector = *rml_o->NewPositionVector;
    *rml_i->CurrentVelocityVector = *rml_o->NewVelocityVector;
    *rml_i->CurrentAccelerationVector = *rml_o->NewAccelerationVector;
    
    // check if finished
    if(rml_result_ == ReflexxesAPI::RML_FINAL_STATE_REACHED)
      { traj_started = false; traj_step++; }
      
    if(traj_step == traj_steps)
    {
      nh.setParam("moving", false); traj_in = false;
      for(int i=0; i<DOF; i++)
        rml_i->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    // copy UR5 state to MuJoCo
    for(int i=0; i<DOF; i++)
    {
      d->qpos[jI[i].p] = rml_o->NewPositionVector->VecData[i];
      d->qvel[jI[i].v] = rml_o->NewVelocityVector->VecData[i];
    }
  }
  else
  {
    // hold positions
    for(int i=0; i<DOF; i++)
    {
      d->qpos[jI[i].p] = jpos.back()[i];
      d->qvel[jI[i].v] = 0.0;
    }
  }
  
  // publish joint state
  std_msgs::Float32MultiArray jstate_out_;
  jstate_out_.data.resize(DOF*2);
  for(int i=0; i<DOF; i++)
  {
    jstate_out_.data[i] = d->qpos[jI[i].p];
    jstate_out_.data[DOF+i] = d->qvel[jI[i].v];
  }
  jstate_pub.publish(jstate_out_);
}



//// ROS

void UR5::jpos_cb(const std_msgs::Float32MultiArray& msg)
{
  int nsteps_ = msg.data.size() / DOF;
  
  if(nsteps_ > 0)
  {
    // target joint positions
    jpos.clear(); int i_ = 0;
    for(int i=0; i<nsteps_; i++)
    {
      std::vector<double> d_(DOF);
      for(int j=0; j<DOF; j++) d_[j] = msg.data[i_++];
      jpos.push_back(d_);
    }
    
    // target joint velocities
    double vC_ = 1.0;
    if(msg.data.size()%DOF == 1) vC_ = msg.data.back();
    jvel.clear();
    for(int i=0; i<nsteps_-1; i++)
    {
      std::vector<double> d_(DOF);
      for(int j=0; j<DOF; j++) d_[j] = vC_ * (jpos[i+1][j]-jpos[i][j]);
      jvel.push_back(d_);
    }
    jvel.push_back(std::vector<double>(DOF, 0.0));
    
    traj_step = 0; traj_steps = nsteps_; traj_in = true;
  }  
}
