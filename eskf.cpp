#include "eskf.hpp"

void eskf::updateErrorState(Eigen::Vector<double, 18>     & x,
                            Eigen::Vector<double, 18>     & dx,
                            Eigen::Matrix<double, 18, 18> & P,
                            Eigen::Matrix<double, OB_LEN, OB_LEN> & V)
{
    Eigen::Matrix<double, 18, OB_LEN> K;
    Eigen::Matrix<double, OB_LEN, 18> H;
    Eigen::Vector<double, OB_LEN> y;
    Eigen::Matrix<double, OB_LEN, OB_LEN> I = Eigen::MatrixXd::Identity(18, 18);

    y = observe(x);
    H = jacobH(x);
    K = P * H.transpose() * (H*P*H.transpose() + V);
    dx = K * (y - H);
    P = (I - K*H) * P;
}

Eigen::Matrix<double, OB_LEN, 18> eskf::jacobH(const Eigen::Vector<double, 18> & x)
{
    Eigen::Matrix<double, OB_LEN, 19> hx;
    Eigen::Matrix<double, 19, 18> Xdx;

    hx  = jacobhx(x);
    Xdx = jacobXdx(x);
    return hx*Xdx;
}

Eigen::Matrix<double, OB_LEN, 19> jacobhx(const Eigen::Vector<double, 18> & x)
{
    Eigen::Matrix<double, OB_LEN, 18> hx;
}

Eigen::Matrix<double, 19, 18> jacobXdx(const Eigen::Vector<double, 18> & x)
{
    Eigen::Matrix<double, 19, 18> Xdx = ;
    
}