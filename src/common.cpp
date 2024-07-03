#include "../include/common.hpp"
#include <cmath>

Eigen::Vector4d quatMultiply(const Eigen::Vector4d & p, const Eigen::Vector4d & q)
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
    if (product.norm() == 0) {
        return product;
    }
    product = product / product.norm();
    return product;
}