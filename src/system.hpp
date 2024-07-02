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

    // true state (only for simulation)
    Eigen::Vector<double, 18> xt;

    // error state
    Eigen::Vector<double, 18> dx;

    // dt (high freq & low freq)
    const double dt_high = 0.01;
    const double dt_low  = 0.1;

    // gravity
    const double g = 9.8;

    // anglar velocity (only for simulation, using for updating true state)
    const double omega_p = 0.1;  // trajectory (the object's trajectory is in simple harmonic motion with respect to each axis)
    const double omega_r = 0.1;  // rotation   (the object's rotation   is in simple harmonic motion with respect to each axis)

    void initialize();
    void updateTrueState(Eigen::Vector<double, 18> & xt, const double t)

}

#endif // SYSTEM_HPP_