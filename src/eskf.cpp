#include <cmath>
#include "../include/eskf.hpp"
#include "../include/system.hpp"

void eskf::updateErrorState(Eigen::Vector<double, 18>     & x,
                            Eigen::Vector<double, 18>     & dx,
                            Eigen::Matrix<double, 18, 18> & P,
                            Eigen::Matrix<double, OB_LEN, OB_LEN> & V)
{
    Eigen::Matrix<double, 18, OB_LEN> K;
    Eigen::Matrix<double, OB_LEN, 18> H;
    Eigen::Vector<double, OB_LEN> y;
    Eigen::Matrix<double, 18, 18> I = Eigen::MatrixXd::Identity(18, 18);
    Eigen::Vector<double, OB_LEN> hx_hat;

    y = observe(x);
    H = jacobH(x);
    K = P * H.transpose() * (H*P*H.transpose() + V);
    dx = K * (y - hx_hat);
    P = (I - K*H) * P;

    
}

void eskf::updateCovarianceMatrix(Eigen::Matrix<double, 18, 18> & P)
{
    P = s::Fx * P * s::Fx.transpose() + s::Qi;
}

Eigen::Matrix<double, OB_LEN, 18> eskf::jacobH(const Eigen::Vector<double, 18> & x)
{
    Eigen::Matrix<double, OB_LEN, 19> hx;
    Eigen::Matrix<double, 19, 18> Xdx;

    hx  = jacobhx(x);
    Xdx = jacobXdx(x);
    return hx*Xdx;
}

Eigen::Matrix<double, OB_LEN, 19> eskf::jacobhx(const Eigen::Vector<double, 18> & x)
{
    Eigen::Matrix<double, OB_LEN, 19> hx;
    return hx;
}

Eigen::Matrix<double, 19, 18> eskf::jacobXdx(const Eigen::Vector<double, 18> & x)
{
    // alias
    const double & qx = x[6];
    const double & qy = x[7];
    const double & qz = x[8];
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
