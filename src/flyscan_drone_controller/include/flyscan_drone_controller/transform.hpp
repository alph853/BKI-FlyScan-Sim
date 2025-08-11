#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>


static const auto NED_ENU_Q = Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
);


static const auto FRD_FLU_Q = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
);
