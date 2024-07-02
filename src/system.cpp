#include "../include/system.hpp"
#include "../include/common.hpp"
#include <cmath>

void system::initialize()
{
    //    p        v        q        ab       ωb       g
    x  << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g;
    xt << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g;
    dx << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

}

void system::updateTrueState(Eigen::Vector<double, 18> & xt, const double t)
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

    qt << std::sqrt(1 - x.segment(6, 3).norm()), x.segment(6, 3);
    q_dot = quatMultiply(qt, w_v) * 0.5;
    qt_new = qt + q_dot * dt_low;
    qt_new = qt_new / qt_new.norm();

    

    xt << p_x, p_y, p_z, 
          v_x, v_y, v_z,
          qt_new[1], qt_new[2], qt_new[3]; 


}