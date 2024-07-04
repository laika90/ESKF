#include <cmath>
#include "../include/eskf.hpp"
#include "../include/system.hpp"

void eskf::updateErrorState()
{
    Eigen::Matrix<double, 18, 9> K;
    Eigen::Matrix<double, 9, 18> H;
    Eigen::Vector<double, 9> y;
    Eigen::Matrix<double, 18, 18> I = Eigen::MatrixXd::Identity(18, 18);

    y = system_user::observe();
    H = jacobH();
    K = system_user::P * H.transpose() * (H*system_user::P*H.transpose() + system_user::V);
    system_user::dx = K * (y - system_user::hx_hat()); 
    system_user::P  = (I - K*H) * system_user::P;

    
}

void eskf::updateCovarianceMatrix(Eigen::Matrix<double, 18, 18> & P)
{
    P = system_user::Phai * system_user::P * system_user::Phai.transpose() + system_user::Qi;
}

Eigen::Matrix<double, 9, 18> eskf::jacobH()
{
    Eigen::Matrix<double, 9, 19> hx;
    Eigen::Matrix<double, 19, 18> Xdx;

    hx  = jacobhx();
    Xdx = jacobXdx();
    return hx*Xdx;
}

Eigen::Matrix<double, 9, 19> eskf::jacobhx()
{
    // alias
    const double & qx  = system_user::x[6];
    const double & qy  = system_user::x[7];
    const double & qz  = system_user::x[8];
    const double   qw  = std::sqrt(1 - (qx*qx + qy*qy + qz*qz));
    const double & abx = system_user::x[9];
    const double & aby = system_user::x[10];
    const double & abz = system_user::x[11];
    const double & g   = system_user::x[17];
    const double & ax  = system_user::a_nominal[0];
    const double & ay  = system_user::a_nominal[1];
    const double & az  = system_user::a_nominal[2];

    Eigen::Matrix<double, 9, 19> hx = Eigen::Matrix<double, 9, 19>::Zero(9, 19);

    hx(0, 0)  =  1;
    hx(1, 1)  =  1;
    hx(2, 2)  =  1;

    hx(3, 6)  =  2*qw*ax + 2*qz*ay - 2*qy*(az-g); // qw
    hx(3, 7)  =  2*qx*ax + 2*qy*ay + 2*qz*(az-g); // qx
    hx(3, 8)  = -2*qy*ax + 2*qx*ay - 2*qw*(az-g); // qy
    hx(3, 9)  = -2*qz*ax + 2*qw*ay + 2*qx*(az-g); // qz
    hx(3, 10) =  1;
    hx(3, 18) = -2*(qx*qz - qw*qy);  

    hx(4, 6)  = -2*qz*ax + 2*qw*ay + 2*qx*(az-g); // qw
    hx(4, 7)  =  2*qy*ax - 2*qx*ay + 2*qw*(az-g); // qx
    hx(4, 8)  =  2*qx*ax + 2*qy*ay + 2*qz*(az-g); // qy
    hx(4, 9)  = -2*qw*ax - 2*qz*ay + 2*qy*(az-g); // qz
    hx(4, 11) =  1;
    hx(4, 18) = -2*(qy*qz + qw*qx);

    hx(5, 6)  =  2*qy*ax - 2*qx*ay - 2*qw*(az-g); // qw
    hx(5, 7)  =  2*qz*ax - 2*qw*ay - 2*qx*(az-g); // qx
    hx(5, 8)  =  2*qw*ax + 2*qz*ay - 2*qy*(az-g); // qy
    hx(5, 9)  =  2*qx*ax + 2*qy*ay + 2*qz*(az-g); // qz
    hx(5, 12) =  1;
    hx(5, 18) = -(qw*qw - qx*qx - qy*qy + qz*qz);

    hx(6, 13) = -1;
    hx(7, 14) = -1;
    hx(8, 15) = -1;

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

void eskf::updateNominal(Eigen::Vector<double, 18> & x, const Eigen::Vector<double, 18> & dx)
{
    for(int i = 0; i < x.rows(); ++i)
    {
        if ((i < 6) || (i > 8)) { x[i] += dx[i]; }
    } 

    Eigen::Vector4d q;
    Eigen::Vector4d dq;
    Eigen::Vector3d dtheta;
    Eigen::Vector4d q_new;
    
    q      << std::sqrt(1 - x.segment(6, 3).norm()), x.segment(6, 3);
    dtheta = dx.segment(6, 3);
    dq     = dthetaTodq(dtheta);
    q_new  = quatMultiply(q, dq);
    x[6] = q_new[1];
    x[7] = q_new[2];
    x[8] = q_new[3];
}

Eigen::Vector4d eskf::dthetaTodq(const Eigen::Vector3d & dtheta)
{
    Eigen::Vector4d dq;
    Eigen::Vector3d u;
    double dtheta_norm = dtheta.norm();
    u = dtheta / dtheta_norm;

    dq << std::cos(dtheta_norm/2), u[0]*std::sin(dtheta_norm/2), u[1]*std::sin(dtheta_norm/2), u[2]*std::sin(dtheta_norm/2);

    return dq; 
}

void eskf::initializeErrorState(Eigen::Vector<double, 18> & dx, Eigen::Matrix<double, 18, 18> & P)
{
    dx = Eigen::Vector<double, 18>::Zero(18, 1);
    // P = P; (G is assumed to be the unit matrix)
}
