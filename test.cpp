#include <iostream>
#include <Eigen/Core>

int main() 
{
    Eigen::Matrix<double, 2, 3> m = Eigen::Matrix<double, 2, 3>::Identity(2,3);
    std::cout << m << std::endl;
}