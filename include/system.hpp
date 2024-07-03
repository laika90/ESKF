#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <Eigen/Core>
#include <random>

namespace system_user
{
    #define OB_LEN 3

    // transition matrix for state
    extern Eigen::Matrix<double, 18, 18> A;
    extern Eigen::Matrix<double, 18, 18> phai;

    // transition matrix for error state
    extern Eigen::Matrix<double, 18, 18> Fx;
    
    // variance of impluse
    extern Eigen::Matrix<double, 18, 18> Qi;
    
    // state
    extern Eigen::Vector<double, 18> x;

    // true state (only for simulation)
    extern Eigen::Vector<double, 18> xt;

    // error state
    extern Eigen::Vector<double, 18> dx;

    // dt (high freq & low freq)
    extern const double dt_high;
    extern const double dt_low;

    // gravity
    extern const double g;

    // anglar velocity (only for simulation, using for updating true state)
    extern const double omega_p;  // trajectory (the object's trajectory is in simple harmonic motion with respect to each axis)
    extern const double omega_r;  // rotation   (the object's rotation   is in simple harmonic motion with respect to each axis)

    // covariance of sensor noise (For simplicity, a single random number generator complements the three axes)
    extern const double v_aw;
    extern const double v_ww;

    // distribution generator
    extern std::random_device rd_for_aw;  // for sensor noise a_w
    extern std::random_device rd_for_ww;  // for sensor noise omega_w
    extern std::mt19937 gen_for_aw;
    extern std::mt19937 gen_for_ww;
    extern std::normal_distribution<> dist_aw;
    extern std::normal_distribution<> dist_ww;

    void updateTrueState(Eigen::Vector<double, 18> & xt, const double t);
}

#endif // SYSTEM_HPP_