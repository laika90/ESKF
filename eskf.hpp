#ifndef ESKF_HPP_
#define ESFK_HPP_

#include <Eigen/Core>

#define OB_LEN 3

namespace eskf 
{
    void updateErrorState(Eigen::Vector<double, 18>     & x,
                          Eigen::Vector<double, 18>     & dx,
                          Eigen::Matrix<double, 18, 18> & P);

    Eigen::Matrix<double, OB_LEN, 18> jacobH(const Eigen::Vector<double, 18> & x);

    Eigen::Vector<double, OB_LEN> observe(const Eigen::Vector<double, 18> & x);

    void updateNominal(Eigen::Vector<double, 18> & x);

    void initializeErrorState(Eigen::Vector<double, 18>     & dx,
                              Eigen::Matrix<double, 18, 18> & P);
}

#endif // EFKF_HPP