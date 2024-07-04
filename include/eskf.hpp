#ifndef ESKF_HPP_
#define ESFK_HPP_

#include <Eigen/Core>
#include "system.hpp"

namespace eskf 
{
    void updateErrorState();

    void updateCovarianceMatrix(Eigen::Matrix<double, 18, 18> & P);

    Eigen::Matrix<double, 9, 18> jacobH();

    Eigen::Matrix<double, 9, 19> jacobhx();

    Eigen::Matrix<double, 19, 18> jacobXdx();

    void updateNominal(Eigen::Vector<double, 18> & x, const Eigen::Vector<double, 18> & dx);

    Eigen::Vector4d dthetaTodq(const Eigen::Vector3d & dtheta);

    void initializeErrorState(Eigen::Vector<double, 18>     & dx,
                              Eigen::Matrix<double, 18, 18> & P);
}

#endif // ESKF_HPP