#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <string>
#include "system.hpp"
#include "eskf.hpp"
#include "common.hpp"

int main()
{
    std::string   output_file_px_nominal = "data/px_nominal.dat";
    std::string   output_file_px_true    = "data/px_true.dat";
    std::ofstream px_nominal_handle (output_file_px_nominal);
    std::ofstream px_true_handle    (output_file_px_true);

    std::string delim = " ";

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
            eskf::initializeErrorState();
            counter = 0;
            std::cout << "----- observation ------" << std::endl;
        }
        std::cout << "###############################" << std::endl;
        std::cout << system_user::x << std::endl;
        std::cout << "###############################" << std::endl;

        // save data
        px_nominal_handle << t << delim << system_user::x[0]  << std::endl;
        px_true_handle    << t << delim << system_user::xt[0] << std::endl;

        if (t > limit) { break; }
    }
    std::cout << "xt" << std::endl;
    std::cout << system_user::xt << std::endl;
}
