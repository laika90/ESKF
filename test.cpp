#include <iostream>
#include <Eigen/Core>
#include <cmath>

Eigen::Vector4d dthetaTodq(Eigen::Vector3d & dtheta)
{
    Eigen::Vector4d dq;
    Eigen::Vector3d u;
    double dtheta_norm = dtheta.norm();
    u = dtheta / dtheta_norm;

    dq << std::cos(dtheta_norm/2), u[0]*std::sin(dtheta_norm/2), u[1]*std::sin(dtheta_norm/2), u[2]*std::sin(dtheta_norm/2);

    return dq; 
}

Eigen::Vector4d quatMultiply(Eigen::Vector4d & p, Eigen::Vector4d & q)
{
    // alias
    double pw = p[0];
    double px = p[1];
    double py = p[2];
    double pz = p[3];
    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    Eigen::Vector4d product;
    product << pw*qw - px*qx - py*qy - pz*qz,
               pw*qx + px*qw + py*qz - pz*qy,
               pw*qy - px*qz + py*qw + pz*qx,
               pw*qz + px*qy - py*qx + pz*qw;
    product = product / product.norm();
    return product;
}



// int main() 
// {
//     Eigen::Vector3d dtheta(0.01, 0.01, 0.01);
//     Eigen::Vector4d dq;
//     Eigen::Vector4d q(1, 0, 0, 0);
//     dq = dthetaTodq(dtheta);
//     q = quatMultiply(q, dq);
//     std::cout << dq << std::endl;
//     std::cout << q << std::endl;
// }

double 

int main()
{

}