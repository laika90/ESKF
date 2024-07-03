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
    std::mt19937 gen_for_aw(rd_for_aw());
    std::mt19937 gen_for_ww(rd_for_ww());
    std::normal_distribution<> dist_aw(0, v_aw);
    std::normal_distribution<> dist_ww(0, v_ww);

    //                            p        v        q        ab       ωb       g
    Eigen::Vector<double, 18> x  (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);
    Eigen::Vector<double, 18> xt (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);
    Eigen::Vector<double, 18> dx (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void system_user::updateTrueState(Eigen::Vector<double, 18> & xt, const double t)
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

    xt << p_x, p_y, p_z, 
          v_x, v_y, v_z,
          qt_new[1], qt_new[2], qt_new[3],
          abx_new, aby_new, abz_new,
          wbx_new, wby_new, wbz_new,
          xt[15], xt[16], xt[17];
}