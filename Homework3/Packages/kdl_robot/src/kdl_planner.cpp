#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _trajRadius, Eigen::Vector3d _trajInit)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    trajEnd_ = _trajInit;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, double _trajRadius, Eigen::Vector3d _trajInit)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    trajEnd_ = _trajInit;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, double _trajRadius, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::set_all(double _trajDuration, double _accDuration, 
                double _trajRadius, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    this->trajDuration_ = _trajDuration;
    this->accDuration_ = _accDuration;
    this->trajInit_ = _trajInit;
    this->trajRadius_ = _trajRadius;
    this->trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time ,char* path, char* vel_prof)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  s_struct curv_abs_;

  if(strcmp(vel_prof,"cubic")==0)
    curv_abs_ = KDLPlanner::cubic_polinomial(time);
  else if(strcmp(vel_prof,"trapez")==0)
    curv_abs_ = KDLPlanner::trapezoidal_vel(time,accDuration_);
  else 
    std::cout<<"ERRORE ABS"<<std::endl;

  if(strcmp(path,"circle")==0){
    traj.pos[0] = trajInit_[0];
    traj.pos[1] = trajInit_[1] - trajRadius_*cos(2*M_PI*curv_abs_.s_);
    traj.pos[2] = trajInit_[2] - trajRadius_*sin(2*M_PI*curv_abs_.s_);

    traj.vel[0] = 0;
    traj.vel[1] = 2*M_PI*trajRadius_*curv_abs_.s_dot_*sin(2*M_PI*curv_abs_.s_);
    traj.vel[2] = -2*M_PI*trajRadius_*curv_abs_.s_dot_*cos(2*M_PI*curv_abs_.s_);

    traj.acc[0] = 0;
    traj.acc[1] = 2*M_PI*trajRadius_*curv_abs_.s_ddot_*sin(2*M_PI*curv_abs_.s_)+4*M_PI*M_PI*trajRadius_*std::pow(curv_abs_.s_dot_,2)*cos(2*M_PI*curv_abs_.s_);
    traj.acc[2] = -2*M_PI*trajRadius_*curv_abs_.s_ddot_*cos(2*M_PI*curv_abs_.s_)+4*M_PI*M_PI*trajRadius_*std::pow(curv_abs_.s_dot_,2)*sin(2*M_PI*curv_abs_.s_);
  }
  else if(strcmp(path,"linear")==0){
    traj.pos = trajInit_ + (curv_abs_.s_ * (trajEnd_ - trajInit_));
    traj.vel = curv_abs_.s_dot_ * (trajEnd_ - trajInit_);
    traj.acc = curv_abs_.s_ddot_ * (trajEnd_ - trajInit_);
  }else{
    std::cout<<"ERRORE PATH"<<std::endl;
  }
  return traj; 

}

s_struct KDLPlanner::trapezoidal_vel(double time, double tc){

  s_struct s_abs; 

  accDuration_= tc;

  double qc_ddot_ = 5/(std::pow(trajDuration_,2));

  if(time <= accDuration_)
  {
    s_abs.s_ = 0 + 0.5*qc_ddot_*std::pow(time,2);
    s_abs.s_dot_ = qc_ddot_*time;
    s_abs.s_ddot_ = qc_ddot_;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s_abs.s_ = 0 + qc_ddot_*accDuration_*(time-accDuration_/2);
    s_abs.s_dot_ = qc_ddot_*accDuration_;
    s_abs.s_ddot_ = 0; 
  }
  else
  {
    s_abs.s_ = 1 - 0.5*qc_ddot_*std::pow(trajDuration_-time,2);
    s_abs.s_dot_ = qc_ddot_*(trajDuration_-time);
    s_abs.s_ddot_ = -qc_ddot_;
  }

  return s_abs;

}

s_struct KDLPlanner::cubic_polinomial(double time){

  s_struct s_abs;
  
  //offline value
  double a0 = 0;
  double a1 = 0;
  double a2 = 3/std::pow(trajDuration_,2);
  double a3 = -2/std::pow(trajDuration_,3);

  s_abs.s_ = a3*std::pow(time,3) + a2*std::pow(time,2) + a1*time + a0; 
  s_abs.s_dot_ = 3*a3*std::pow(time,2) + 2*a2*time + a1;
  s_abs.s_ddot_ = 6*a3*time + 2*a2;

  return s_abs;

}
