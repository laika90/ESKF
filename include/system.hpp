#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <Eigen/Core>
#include <random>

namespace system_user
{
    // #define OBSERVE_LENGTH 9

    // transition matrix for state
    extern Eigen::Matrix<double, 18, 18> A;
    extern Eigen::Matrix<double, 18, 18> Phai;

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

    // acceleration nominal
    extern Eigen::Vector3d a_nominal;

    // to hold other states needed to consider obsercvation (only for simulation)
    extern Eigen::Vector<double, 6> other_true_state;

    // Rotation Matrix (quaternion to rotation matrix) true state
    extern Eigen::Matrix3d Rt;

    // covariance matrix of error state dx
    extern Eigen::Matrix<double, 18, 18> P;

    // covariance matrix of observation noise
    extern Eigen::Matrix<double, 9, 9> V;

    // dt (high freq & low freq)
    extern const double dt_high;
    extern const double dt_low;

    // gravity
    extern const double g;

    // anglar velocity (only for simulation, using for updating true state)
    extern const double omega_p;  // trajectory (the object's trajectory is in simple harmonic motion with respect to each axis)
    extern const double omega_r;  // rotation   (the object's rotation   is in simple harmonic motion with respect to each axis)

    // covariance of sensor bias noise (For simplicity, a single random number generator complements the three axes)
    extern const double v_aw;
    extern const double v_ww;

    // covariance of sensor noise (For simplicity, a single random number generator complements the three axes)
    extern const double v_an;
    extern const double v_wn;
    extern const double v_pn;

    // distribution generator
    extern std::random_device rd_for_aw;  // for sensor bias noise a_w
    extern std::random_device rd_for_ww;  // for sensor bias noise omega_w
    extern std::random_device rd_for_an;  // for sensor noise a_n
    extern std::random_device rd_for_wn;  // for sensor noise omega_n
    extern std::random_device rd_for_pn;  // for sensor noise p_n (used for GPS)
    extern std::mt19937 gen_for_aw;
    extern std::mt19937 gen_for_ww;
    extern std::mt19937 gen_for_an;
    extern std::mt19937 gen_for_wn;
    extern std::mt19937 gen_for_pn;
    extern std::normal_distribution<> dist_aw;
    extern std::normal_distribution<> dist_ww;
    extern std::normal_distribution<> dist_an;
    extern std::normal_distribution<> dist_wn;
    extern std::normal_distribution<> dist_pn;

    void updateTrueState(const double t);
    void updateRotationMatrix(Eigen::Vector4d & quat);
    Eigen::Vector<double, 9> observeWithoutNoise(const Eigen::Vector<double, 18> & state);
    Eigen::Vector<double, 9> observe();
    Eigen::Vector<double, 9> hx_hat();
}

#endif // SYSTEM_HPP_