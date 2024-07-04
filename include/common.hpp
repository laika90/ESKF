#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <Eigen/Core>

Eigen::Vector4d quatMultiply(const Eigen::Vector4d & p, const Eigen::Vector4d & q);
Eigen::Vector4d thetaToq(const Eigen::Vector3d & theta);


#endif // COMMON_HPP_