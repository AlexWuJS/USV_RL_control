#ifndef POSE6D_H
#define POSE6D_H

#include <Eigen/Dense>

namespace fast_lio {

struct Pose6D {
    double x, y, z;
    double roll, pitch, yaw;
    
    Pose6D() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    Pose6D(double _x, double _y, double _z, double _roll, double _pitch, double _yaw) 
        : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {}
    
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        matrix.block<3,3>(0,0) = rotation;
        matrix.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);
        return matrix;
    }
};

} // namespace fast_lio

#endif // POSE6D_H 