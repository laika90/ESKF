#include "system.hpp"
#include <cmath>

void system::initialize()
{
    //    p        v        q           ab       ωb       g
    x  << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g;
    xt << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g;
    dx << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

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

}