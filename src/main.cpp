#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include "../include/system.hpp"

int main()
{
    double t = 0;

    while (true)
    {
        system_user::updateTrueState(system_user::xt, t);
        std::cout << "####################" << std::endl;
        std::cout << system_user::xt << std::endl;
        t += system_user::dt_high;
        if (t > 1) { break; }
    }
}
