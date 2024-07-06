#include "system.hpp"
#include "common.hpp"
#include <cmath>

namespace system_user
{
    const double dt_high = 0.01;
    const double dt_low  = 0.1;
    const double g = -9.8;
    const double omega_p = 0.1; 
    const double omega_r = 0.1;
    const double v_aw = 0.1;
    const double v_ww = 0.1;
    const double v_pn = 0.1;
    const double v_vn = 0.1;
    const double v_an = 0.1;
    const double v_wn = 0.1;

    Eigen::Vector<double, 6> V_vec (v_pn, v_pn, v_pn, v_vn, v_vn, v_vn);
    Eigen::Matrix<double, 6, 6> V =  V_vec.asDiagonal();

    Eigen::Vector3d a_nominal = Eigen::Vector3d::Zero();

    std::random_device rd_for_aw;  
    std::random_device rd_for_ww;  
    std::random_device rd_for_pn;
    std::random_device rd_for_vn;
    std::random_device rd_for_an;
    std::random_device rd_for_wn;
    std::mt19937 gen_for_aw(rd_for_aw());
    std::mt19937 gen_for_ww(rd_for_ww());
    std::mt19937 gen_for_pn(rd_for_pn());
    std::mt19937 gen_for_vn(rd_for_vn());
    std::mt19937 gen_for_an(rd_for_an());
    std::mt19937 gen_for_wn(rd_for_wn());
    std::normal_distribution<> dist_aw(0, v_aw);
    std::normal_distribution<> dist_ww(0, v_ww);
    std::normal_distribution<> dist_pn(0, v_pn);
    std::normal_distribution<> dist_vn(0, v_vn);
    std::normal_distribution<> dist_an(0, v_an);
    std::normal_distribution<> dist_wn(0, v_wn);

    //                            p        v        q        ab       ωb       g
    Eigen::Vector<double, 18> x  (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);
    Eigen::Vector<double, 18> xt (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);
    Eigen::Vector<double, 18> dx (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    //                                            a        omega
    Eigen::Vector<double, 6> other_nominal_state (0, 0, 0, 0, 0, 0);
    Eigen::Vector<double, 6> other_true_state    (0, 0, 0, 0, 0, 0);

    Eigen::Matrix3d Rt = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rn = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 18, 18> Phai = Eigen::Matrix<double, 18, 18>::Identity();

    Eigen::Matrix<double, 18, 18> Qi = Eigen::Matrix<double, 18, 18>::Identity() * 0.01;
    Eigen::Matrix<double, 18, 18> P  = Eigen::Matrix<double, 18, 18>::Identity() * 0.01;
}

void system_user::updateTrueState(const double t)
{
    const double p_x =   std::sin(  omega_p*t) + 0.5*xt[15]*t*t;
    const double p_y = 2*std::sin(2*omega_p*t) + 0.5*xt[16]*t*t;
    const double p_z = 3*std::sin(3*omega_p*t) + 0.5*xt[17]*t*t;
    const double v_x =   omega_p*std::cos(  omega_p*t) + xt[15]*t;
    const double v_y = 4*omega_p*std::cos(2*omega_p*t) + xt[16]*t;
    const double v_z = 9*omega_p*std::cos(3*omega_p*t) + xt[17]*t;
    const double w_x =   std::sin(omega_r*t);
    const double w_y = 2*std::sin(omega_r*t);
    const double w_z = 3*std::sin(omega_r*t);

    Eigen::Vector4d qt;
    Eigen::Vector4d w_v(0, w_x, w_y, w_z);
    Eigen::Vector3d wt (w_x, w_y, w_z);
    Eigen::Vector4d q_dot;
    Eigen::Vector4d qt_new;

    qt << std::sqrt(1 - xt.segment(6, 3).norm()), xt.segment(6, 3);
    // q_dot = quatMultiply(qt, w_v) * 0.5;
    // qt_new = qt + q_dot * dt_high;
    q_dot = thetaToq(wt*dt_high);
    qt_new = quatMultiply(qt, q_dot);
    qt_new.normalize();

    const double abx_new = xt[9]  + dist_aw(gen_for_aw) * dt_high;
    const double aby_new = xt[10] + dist_aw(gen_for_aw) * dt_high;
    const double abz_new = xt[11] + dist_aw(gen_for_aw) * dt_high;
    const double wbx_new = xt[12] + dist_ww(gen_for_ww) * dt_high;
    const double wby_new = xt[13] + dist_ww(gen_for_ww) * dt_high;
    const double wbz_new = xt[14] + dist_ww(gen_for_ww) * dt_high;

    const double ax = -   omega_p*omega_p*std::sin(  omega_p*t) + xt[15];
    const double ay = - 8*omega_p*omega_p*std::sin(2*omega_p*t) + xt[16];
    const double az = -27*omega_p*omega_p*std::sin(3*omega_p*t) + xt[17];

    xt << p_x, p_y, p_z, 
          v_x, v_y, v_z,
          qt_new[1], qt_new[2], qt_new[3],
          abx_new, aby_new, abz_new,
          wbx_new, wby_new, wbz_new,
          xt[15], xt[16], xt[17];
    
    // update other true state and rotation matrix
    other_true_state << ax, ay, az, w_x, w_y, w_z;
    updateRotationMatrix(qt_new, Rt);
}

void system_user::updateRotationMatrix(const Eigen::Vector4d & quat, Eigen::Matrix3d & R)
{
    // alias
    const double & qw = quat[0];
    const double & qx = quat[1];
    const double & qy = quat[2];
    const double & qz = quat[3];

    R << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz),             2*(qx*qz + qw*qy),
         2*(qx*qy + qz*qw),             qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
         2*(qx*qz - qw*qy),             2*(qy*qz + qx*qw),             qw*qw - qx*qx - qy*qy + qz*qz;
}

Eigen::Vector<double, 6> system_user::observeWithoutNoise(const Eigen::Vector<double, 18> & state)
{
    // this function can be used in both true state and nominal state

    // position 
    const Eigen::Vector3d pm_without_noise (state[0], state[1], state[2]);

    // velocity
    const Eigen::Vector3d vm_without_noise (state[3], state[4], state[5]);

    Eigen::Vector<double, 6> y_without_noise;
    y_without_noise << pm_without_noise, vm_without_noise;

    return y_without_noise;
}

Eigen::Vector<double, 6> system_user::observe()
{
    // observation without noise
    const Eigen::Vector<double, 6> y_without_noise = observeWithoutNoise(xt);

    // noise 
    const Eigen::Vector<double, 6> noise (dist_pn(gen_for_pn), dist_pn(gen_for_pn), dist_pn(gen_for_pn),
                                          dist_vn(gen_for_vn), dist_vn(gen_for_vn), dist_vn(gen_for_vn));
    
    // observation with noise
    Eigen::Vector<double, 6> y;
    y = y_without_noise + noise;
    
    return y;
}

Eigen::Vector<double, 6> system_user::hx_hat()
{
    // it functions as prediction if observeWithoutNoise is adapted to nominal state
    return observeWithoutNoise(x);
}

void system_user::updatePhai()
{
    // alias 
    const double & ax = other_nominal_state[0];
    const double & ay = other_nominal_state[1];
    const double & az = other_nominal_state[2];
    const Eigen::Vector3d & w_nominal = other_nominal_state.segment(3, 3);

    Eigen::Matrix3d a_cross;
    a_cross <<  0, -az,  ay,
                az,  0, -ax,
               -ay, ax,  0;
    Eigen::Matrix3d Ra = Rn * a_cross;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Rwdt;
    Eigen::Vector4d wdt_quat = thetaToq(w_nominal*dt_high);
    updateRotationMatrix(wdt_quat, Rwdt);

    // only first time. invariant part.
    if (system_user::Phai(0, 3) != 0) 
    {
        system_user::Phai.block<3, 3>(0, 3)  = I * dt_high;
        system_user::Phai.block<3, 3>(0, 15) = 0.5 * I * dt_high * dt_high;
        system_user::Phai.block<3, 3>(3, 15) = I * dt_high;
        system_user::Phai.block<3, 3>(6, 12) = -I * dt_high;
    }
    
    system_user::Phai.block<3, 3>(0, 6)  = -0.5 * Ra * dt_high * dt_high;
    system_user::Phai.block<3, 3>(0, 9)  = -0.5 * Rn * dt_high * dt_high;
    system_user::Phai.block<3, 3>(3, 6)  = -0.5 * Ra * dt_high;
    system_user::Phai.block<3, 3>(3, 9)  = -0.5 * Rn * dt_high;
    system_user::Phai.block<3, 3>(3, 12) =  0.5 * Ra * dt_high * dt_high;
    system_user::Phai.block<3, 3>(6, 6)  = Rwdt.transpose();
}

Eigen::Vector<double, 6> system_user::getSensorValueWithoutNoise()
{
    // acceleration alias
    const Eigen::Vector3d & at  = other_true_state.segment(0, 3);
    const Eigen::Vector3d & gt  = xt.segment(15, 3);
    const Eigen::Vector3d & abt = xt.segment(9, 3);

    // acceleration 
    Eigen::Vector3d am_without_noise;
    am_without_noise << Rt.transpose() * (at - gt) + abt;

    // angular velocity
    const Eigen::Vector3d wm_without_noise (other_true_state[3] - xt[12], other_true_state[4] - xt[13], other_true_state[5] - xt[14]);

    Eigen::Vector<double, 6> sensor_value_without_noise;
    sensor_value_without_noise << am_without_noise, wm_without_noise;

    return sensor_value_without_noise;
}

Eigen::Vector<double, 6> system_user::getSensorValue()
{
    Eigen::Vector<double, 6> sensor_value_without_noise = getSensorValueWithoutNoise();
    const Eigen::Vector<double, 6> noise (dist_an(gen_for_an), dist_an(gen_for_an), dist_an(gen_for_an),
                                          dist_wn(gen_for_wn), dist_wn(gen_for_wn), dist_wn(gen_for_wn));
    
    return sensor_value_without_noise + noise;
}

void system_user::oneStep(const Eigen::Vector<double, 6> & sensor_value, const double t)
{
    // alias
    const Eigen::Vector3d & am = sensor_value.segment(0, 3);
    const Eigen::Vector3d & wm = sensor_value.segment(3, 3);
    const Eigen::Vector3d & p  = x.segment(0, 3);
    const Eigen::Vector3d & v  = x.segment(3, 3);
    const Eigen::Vector4d   q    (std::sqrt(1 - x.segment(6, 3).norm()), x[6], x[7], x[8]);
    const Eigen::Vector3d & ab = x.segment(9, 3);
    const Eigen::Vector3d & wb = x.segment(12, 3);
    const Eigen::Vector3d & g  = x.segment(15, 3);

    // update
    const Eigen::Vector3d p_new = p + v*dt_high + 0.5*(Rn*(am - ab) + g)*dt_high*dt_high;
    Eigen::Vector3d v_new = v + (Rn*(am - ab) + g) * dt_high; 
    if (t == 0) { v_new << omega_p*std::cos(omega_p*t) + x[15]*t, 4*omega_p*std::cos(2*omega_p*t) + x[16]*t, 9*omega_p*std::cos(3*omega_p*t) + x[17]*t; }

    const Eigen::Vector4d dq    = thetaToq((wm - wb)*dt_high);
          Eigen::Vector4d q_new = quatMultiply(q, dq);
    q_new.normalize();

    x << p_new, v_new, q_new.segment(1, 3), ab, wb, g;
    other_nominal_state << am-ab, wm-wb;
    updateRotationMatrix(q_new, Rn);
    updatePhai();
}