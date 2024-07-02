#ifndef ESKF_HPP_
#define ESFK_HPP_

#include <Eigen/Core>
#include "system.hpp"

namespace s = system;

namespace eskf 
{
    void updateErrorState(Eigen::Vector<double, 18>     & x,
                          Eigen::Vector<double, 18>     & dx,
                          Eigen::Matrix<double, 18, 18> & P,
                          Eigen::Matrix<double, s::OB_LEN, s::OB_LEN> & V);

    void updateCovarianceMatrix(Eigen::Matrix<double, 18, 18> & P);

    Eigen::Matrix<double, s::OB_LEN, 18> jacobH(const Eigen::Vector<double, 18> & x);

    Eigen::Matrix<double, s::OB_LEN, 19> jacobhx(const Eigen::Vector<double, 18> & x);

    Eigen::Matrix<double, 19, 18> jacobXdx(const Eigen::Vector<double, 18> & x);

    Eigen::Vector<double, s::OB_LEN> observe(const Eigen::Vector<double, 18> & x);

    void updateNominal(Eigen::Vector<double, 18> & x, const Eigen::Vector<double, 18> & dx);

    Eigen::Vector4d dthetaTodq(const Eigen::Vector3d & dtheta);

    Eigen::Vector4d quatMultiply(const Eigen::Vector4d & p, const Eigen::Vector4d & q);

    void initializeErrorState(Eigen::Vector<double, 18>     & dx,
                              Eigen::Matrix<double, 18, 18> & P);
}

#endif // EFKF_HPP