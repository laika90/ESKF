#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <Eigen/Core>

namespace system
{
    // transition matrix for state
    Eigen::Matrix<double, 18, 18> A;
    Eigen::Matrix<double, 18, 18> phai;

    // transition matrix for error state
    Eigen::Matrix<double, 18, 18> Fx;
    
    // variance of impluse
    Eigen::Matrix<double, 18, 18> Qi;
    
    // state
    Eigen::Vector<double, 18> x;

    // error state
    Eigen::Vector<double, 18> dx;

}

#endif // SYSTEM_HPP_