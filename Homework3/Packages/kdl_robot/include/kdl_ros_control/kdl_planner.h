#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

struct s_struct{
    double s_=0;
    double s_dot_=0;
    double s_ddot_=0;
};

class KDLPlanner
{

public:

    KDLPlanner(double _maxVel, double _maxAcc);
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //////////////////////////////////
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    KDLPlanner(double _trajDuration, double _trajRadius,
               Eigen::Vector3d _trajInit);
    KDLPlanner(double _trajDuration, double _accDuration, 
                double _trajRadius, Eigen::Vector3d _trajInit);
    KDLPlanner(double _trajDuration, double _accDuration, 
                double _trajRadius, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    
    void set_all(double _trajDuration, double _accDuration, 
                double _trajRadius, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    trajectory_point compute_trajectory(double time,char* path,char* vel);
    s_struct cubic_polinomial(double time);
    s_struct trapezoidal_vel(double time, double tc);

private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_;
    Eigen::Vector3d trajInit_, trajEnd_;
    double trajRadius_;
    trajectory_point p;

};

#endif
