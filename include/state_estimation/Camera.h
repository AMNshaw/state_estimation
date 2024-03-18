#ifndef CAMERA_H
#define CAMERA_H
#pragma once
#include <Eigen/Dense>

class Camera
{
private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double lx_;
    double ly_;
    
    Eigen::Vector3d t_b2c;
    Eigen::Matrix3d R_b2c;
public:
    
    Camera();
    Camera(double f_x, double f_y, double c_x, double c_y);
    ~Camera();
    void setParameters(double f_x, double f_y, double c_x, double c_y);
    double fx();
    double fy();
    double cx();
    double cy();
    double lx();
    double ly();

    Eigen::Vector3d t_B2C();
    Eigen::Matrix3d R_B2C();
};

#endif