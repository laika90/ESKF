#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include "system.hpp"
#include "eskf.hpp"
#include "common.hpp"

int main()
{
    double t = 0;
    int counter = 0;
    double limit = 100;

    while (true)
    {
        system_user::updateTrueState(t);
        Eigen::Vector<double, 6> sensor_value = system_user::getSensorValue();
        system_user::oneStep(sensor_value, t); 
        eskf::updateCovarianceMatrix();

        t += system_user::dt_high;
        ++counter;

        if (counter >= (system_user::dt_low / system_user::dt_high))
        {
            eskf::updateErrorState();
            eskf::updateNominal();
            std::cout << "dx" << std::endl;
            std::cout << system_user::dx << std::endl;
            eskf::initializeErrorState();
            counter = 0;
            std::cout << "----- observation ------" << std::endl;
        }
        std::cout << "###############################" << std::endl;
        std::cout << system_user::x << std::endl;
        // std::cout << system_user::P << std::endl;
        std::cout << "###############################" << std::endl;

        if (t > limit) { break; }
    }
    std::cout << "xt" << std::endl;
    std::cout << system_user::xt << std::endl;
    std::cout << "other true state" << std::endl;
    std::cout << system_user::other_true_state << std::endl;
    std::cout << "other nominal state" << std::endl;
    std::cout << system_user::Rn * (system_user::other_nominal_state.segment(0, 3)) + system_user::x.segment(15, 3)<< std::endl;
    std::cout << "Rt" << std::endl;
    std::cout << system_user::Rt << std::endl;
    std::cout << "Rn" << std::endl;
    std::cout << system_user::Rn << std::endl;
    std::cout << "Rt.T * Rn" << std::endl;
    std::cout << system_user::Rt.transpose() * system_user::Rn << std::endl;
    std::cout << "sensor" << std::endl;
    std::cout << system_user::Rn*system_user::getSensorValue().segment(0,3) << std::endl;
    std::cout << system_user::Rt*system_user::getSensorValue().segment(0,3) << std::endl;

}
