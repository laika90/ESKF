#include "../include/system.hpp"
#include "../include/common.hpp"
#include <cmath>
#include <iostream>

namespace system_user
{
    const double dt_high = 0.01;
    const double dt_low  = 0.1;
    const double g = -9.8;
    const double omega_p = 0.1; 
    const double omega_r = 0.1;
    const double v_aw = 0.01;
    const double v_ww = 0.01;

    std::random_device rd_for_aw;  
    std::random_device rd_for_ww;  
    std::random_device rd_for_an; 
    std::random_device rd_for_wn; 
    std::random_device rd_for_pn;
    std::mt19937 gen_for_aw(rd_for_aw());
    std::mt19937 gen_for_ww(rd_for_ww());
    std::mt19937 gen_for_an(rd_for_an());
    std::mt19937 gen_for_wn(rd_for_wn());
    std::mt19937 gen_for_pn(rd_for_pn());
    std::normal_distribution<> dist_aw(0, v_aw);
    std::normal_distribution<> dist_ww(0, v_ww);
    std::normal_distribution<> dist_an(0, v_an);
    std::normal_distribution<> dist_wn(0, v_wn);
    std::normal_distribution<> dist_pn(0, v_pn);

    //                            p        v        q        ab       ωb       g
    Eigen::Vector<double, 18> x  (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);
    Eigen::Vector<double, 18> xt (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);
    Eigen::Vector<double, 18> dx (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    //                                         a        omega
    Eigen::Vector<double, 6> other_true_state (0, 0, 0, 0, 0, 0);

    Eigen::Matrix3d Rt = Eigen::Matrix3d::Identity();
}

void system_user::updateTrueState(const double t)
{
    const double p_x =   std::sin(  omega_p*t);
    const double p_y = 2*std::sin(2*omega_p*t);
    const double p_z = 3*std::sin(3*omega_p*t);
    const double v_x =   omega_p*std::cos(  omega_p*t);
    const double v_y = 2*omega_p*std::cos(2*omega_p*t);
    const double v_z = 3*omega_p*std::cos(3*omega_p*t);
    const double w_x =   std::sin(omega_r*t);
    const double w_y = 2*std::sin(omega_r*t);
    const double w_z = 3*std::sin(omega_r*t);

    Eigen::Vector4d qt;
    Eigen::Vector4d w_v(0, w_x, w_y, w_z);
    Eigen::Vector4d q_dot;
    Eigen::Vector4d qt_new;

    qt << std::sqrt(1 - xt.segment(6, 3).norm()), xt.segment(6, 3);
    q_dot = quatMultiply(qt, w_v) * 0.5;
    qt_new = qt + q_dot * dt_high;
    std::cout << q_dot << std::endl;
    //qt_new.normalize();

    const double abx_new = xt[9]  + dist_aw(gen_for_aw) * dt_high;
    const double aby_new = xt[10] + dist_aw(gen_for_aw) * dt_high;
    const double abz_new = xt[11] + dist_aw(gen_for_aw) * dt_high;
    const double wbx_new = xt[12] + dist_ww(gen_for_ww) * dt_high;
    const double wby_new = xt[13] + dist_ww(gen_for_ww) * dt_high;
    const double wbz_new = xt[14] + dist_ww(gen_for_ww) * dt_high;

    const double ax = (v_x - xt[3]) / dt_high;
    const double ay = (v_y - xt[4]) / dt_high;
    const double az = (v_z - xt[5]) / dt_high;

    xt << p_x, p_y, p_z, 
          v_x, v_y, v_z,
          qt_new[1], qt_new[2], qt_new[3],
          abx_new, aby_new, abz_new,
          wbx_new, wby_new, wbz_new,
          xt[15], xt[16], xt[17];
    
    other_true_state << ax, ay, az, w_x, w_y, w_z;

    updateRotationMatrix(qt_new);
}

void system_user::updateRotationMatrix(Eigen::Vector4d & quat)
{
    // alias
    const double & qw = quat[0];
    const double & qx = quat[1];
    const double & qy = quat[2];
    const double & qz = quat[3];

    Rt << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz),             2*(qx*qz + qw*qy),
          2*(qx*qy + qz*qw),             qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
          2*(qx*qz - qw*qy),             2*(qy*qz + qx*qw),             qw*qw - qx*qx - qy*qy + qz*qz;
}

Eigen::Vector<double, 9> system_user::observeWithoutNoise(const Eigen::Vector<double, 18> & state)
{
    // this function can be used in both true state and nominal state

    // position 
    const Eigen::Vector3d pm_without_noise (xt[0], xt[1], xt[2]);

    // acceleration alias
    const Eigen::Vector3d & at  = other_true_state.segment(0, 3);
    const Eigen::Vector3d & gt  = xt.segment(15, 3);
    const Eigen::Vector3d & abt = xt.segment(9, 3);

    // acceleration 
    Eigen::Vector3d am_without_noise;
    am_without_noise << Rt.transpose() * (at - gt) + abt;

    // angular velocity
    const Eigen::Vector3d wm_without_noise (other_true_state[3] - xt[12], other_true_state[4] - xt[13], other_true_state[5] - xt[14]);

    Eigen::Vector<double, 9> y_without_noise;
    y_without_noise << pm_without_noise, am_without_noise, wm_without_noise;

    return y_without_noise;
}

Eigen::Vector<double, 9> system_user::observe()
{
    // observation without noise
    const Eigen::Vector<double, 9> y_without_noise = observeWithoutNoise(xt);

    // noise 
    const Eigen::Vector<double, 9> noise (dist_pn(gen_for_pn), dist_pn(gen_for_pn), dist_pn(gen_for_pn),
                                          dist_an(gen_for_an), dist_an(gen_for_an), dist_an(gen_for_an),
                                          dist_wn(gen_for_wn), dist_wn(gen_for_wn), dist_wn(gen_for_wn));
    
    // observation with noise
    Eigen::Vector<double, 9> y;
    y = y_without_noise + noise;
    
    return y;
}

Eigen::Vector<double, 9> system_user::hx_hat()
{
    // it functions as prediction if observeWithoutNoise is adapted to nominal state
    return observeWithoutNoise(x);
}