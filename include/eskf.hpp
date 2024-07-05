#ifndef ESKF_HPP_
#define ESFK_HPP_

#include <Eigen/Core>
#include "system.hpp"

namespace eskf 
{
    void updateErrorState();

    void updateCovarianceMatrix();

    Eigen::Matrix<double, 6, 18> jacobH();

    Eigen::Matrix<double, 6, 19> jacobhx();

    Eigen::Matrix<double, 19, 18> jacobXdx();

    void updateNominal();

    void initializeErrorState();
}

#endif // ESKF_HPP