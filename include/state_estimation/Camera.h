#ifndef CAMERA_H
#define CAMERA_H

class Camera
{
private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double lx_;
    double ly_;
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
};

#endif