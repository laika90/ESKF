#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include "../include/system.hpp"
#include "../include/eskf.hpp"
#include "../include/common.hpp"

int main()
{
    double t = 0;
    int counter = 0;
    double limit = 0.1;

    while (true)
    {
        system_user::updateTrueState(t);
        Eigen::Vector<double, 6> sensor_value = system_user::getSensorValue();
        system_user::oneStep(sensor_value); 
        eskf::updateCovarianceMatrix();

        t += system_user::dt_high;
        ++counter;

        if (counter >= (system_user::dt_low / system_user::dt_high))
        {
            eskf::updateErrorState();
            eskf::updateNominal();
            eskf::initializeErrorState();
            counter = 0;
            std::cout << "----- observation ------" << std::endl;
        }
        std::cout << "###############################" << std::endl;
        std::cout << system_user::x << std::endl;
        std::cout << "###############################" << std::endl;

        if (t > limit) { break; }
    }

    std::cout << system_user::xt << std::endl;
}
