#include <cmath>
#include "../include/eskf.hpp"
#include "../include/system.hpp"
#include "../include/common.hpp"

void eskf::updateErrorState()
{
    Eigen::Matrix<double, 18, 6> K;
    Eigen::Matrix<double, 6, 18> H;
    Eigen::Vector<double, 6> y;
    Eigen::Matrix<double, 18, 18> I = Eigen::MatrixXd::Identity(18, 18);

    y = system_user::observe();
    H = jacobH();
    K = system_user::P * H.transpose() * (H*system_user::P*H.transpose() + system_user::V);
    system_user::dx = K * (y - system_user::hx_hat()); 
    system_user::P  = (I - K*H) * system_user::P;
}

void eskf::updateCovarianceMatrix()
{
    system_user::P = system_user::Phai * system_user::P * system_user::Phai.transpose() + system_user::Qi;
}

Eigen::Matrix<double, 6, 18> eskf::jacobH()
{
    Eigen::Matrix<double, 6, 19> hx;
    Eigen::Matrix<double, 19, 18> Xdx;

    hx  = jacobhx();
    Xdx = jacobXdx();
    return hx*Xdx;
}

Eigen::Matrix<double, 6, 19> eskf::jacobhx()
{
    Eigen::Matrix<double, 6, 19> hx = Eigen::Matrix<double, 6, 19>::Identity(6, 19);
    return hx;
}

Eigen::Matrix<double, 19, 18> eskf::jacobXdx()
{
    // alias
    const double & qx = system_user::x[6];
    const double & qy = system_user::x[7];
    const double & qz = system_user::x[8];
    const double   qw = std::sqrt(1 - (qx*qx + qy*qy + qz*qz));

    Eigen::Matrix<double, 19, 18> Xdx = Eigen::Matrix<double, 19, 18>::Zero(19, 18);
    for (int i = 0; i < Xdx.rows(); ++i)
    {
        if (i < 6) { Xdx(i, i) = 1; }
        else if (i > 9) { Xdx(i, i-1) = 1; }
    }
    Xdx(6, 6) = -0.5*qx;  Xdx(6, 7) = -0.5*qy;  Xdx(6, 8) = -0.5*qz;
    Xdx(7, 6) =  0.5*qw;  Xdx(7, 7) = -0.5*qz;  Xdx(7, 8) =  0.5*qy;
    Xdx(8, 6) =  0.5*qz;  Xdx(8, 7) =  0.5*qw;  Xdx(8, 8) = -0.5*qx;
    Xdx(9, 6) = -0.5*qy;  Xdx(9, 7) =  0.5*qx;  Xdx(9, 8) =  0.5*qw;

    return Xdx;
}

void eskf::updateNominal()
{
    for(int i = 0; i < system_user::x.rows(); ++i)
    {
        if ((i < 6) || (i > 8)) { system_user::x[i] += system_user::dx[i]; }
    } 

    Eigen::Vector4d q;
    Eigen::Vector4d dq;
    Eigen::Vector3d dtheta;
    Eigen::Vector4d q_new;
    
    q      << std::sqrt(1 - system_user::x.segment(6, 3).norm()), system_user::x.segment(6, 3);
    dtheta = system_user::dx.segment(6, 3);
    dq     = thetaToq(dtheta);
    q_new  = quatMultiply(q, dq);
    system_user::x[6] = q_new[1];
    system_user::x[7] = q_new[2];
    system_user::x[8] = q_new[3];
}



void eskf::initializeErrorState()
{
    system_user::dx = Eigen::Vector<double, 18>::Zero(18, 1);
    // system_user::P = system_user::P; (G is assumed to be the unit matrix)
}
