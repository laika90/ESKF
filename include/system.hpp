#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <Eigen/Core>
#include <random>

namespace system_user
{
    // #define OBSERVE_LENGTH 9

    // transition matrix for state
    extern Eigen::Matrix<double, 18, 18> Phai;
    
    // variance of impluse
    extern Eigen::Matrix<double, 18, 18> Qi;
    
    // state
    extern Eigen::Vector<double, 18> x;

    // true state (only for simulation)
    extern Eigen::Vector<double, 18> xt;

    // error state
    extern Eigen::Vector<double, 18> dx;

    // to hold other states needed to consider observation (only for simulation)
    extern Eigen::Vector<double, 6> other_true_state;

    // to hold other states needed to update phai
    extern Eigen::Vector<double, 6> other_nominal_state;

    // Rotation Matrix (quaternion to rotation matrix) true state
    extern Eigen::Matrix3d Rt;

    // Rotation Matrix (quaternion to rotation matrix) nominal state
    extern Eigen::Matrix3d Rn;

    // covariance matrix of error state dx
    extern Eigen::Matrix<double, 18, 18> P;

    // covariance matrix of observation noise
    extern Eigen::Matrix<double, 6, 6> V;

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
    extern std::random_device rd_for_pn;  // for sensor noise p_n (used for GPS)
    extern std::random_device rd_for_vn;  // for sensor noise v_n (used for GPS)
    extern std::random_device rd_for_an;  // for sensor noise a_n
    extern std::random_device rd_for_wn;  // for sensor noise omega_n

    extern std::mt19937 gen_for_aw;
    extern std::mt19937 gen_for_ww;
    extern std::mt19937 gen_for_pn;
    extern std::mt19937 gen_for_vn;
    extern std::mt19937 gen_for_an;
    extern std::mt19937 gen_for_wn;

    extern std::normal_distribution<> dist_aw;
    extern std::normal_distribution<> dist_ww;
    extern std::normal_distribution<> dist_pn;
    extern std::normal_distribution<> dist_vn;
    extern std::normal_distribution<> dist_an;
    extern std::normal_distribution<> dist_wn;

    void updateTrueState(const double t);
    void updateRotationMatrix(const Eigen::Vector4d & quat, Eigen::Matrix3d & R); // used for both nominal and true state
    Eigen::Vector<double, 6> observeWithoutNoise(const Eigen::Vector<double, 18> & state);
    Eigen::Vector<double, 6> observe();
    Eigen::Vector<double, 6> hx_hat();
    void updatePhai();
    Eigen::Vector<double, 6> getSensorValueWithoutNoise();
    Eigen::Vector<double, 6> getSensorValue();
    void oneStep(const Eigen::Vector<double, 6> & sensor_value);
}

#endif // SYSTEM_HPP_