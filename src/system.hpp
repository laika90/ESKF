#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <Eigen/Core>

namespace system
{
    #define OB_LEN 3

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

    // dt (high freq & low freq)
    double dt_high = 0.01;
    double dt_low  = 0.1;

    void initialize();
    Eigen::Vector<double, OB_LEN> hx_hat(const Eigen::Vector<double, 18> & x)

}

#endif // SYSTEM_HPP_