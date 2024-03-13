#include "Camera.h"

Camera::Camera()
{
    fx_ = 565.6008952774197;
    fy_ = 565.6008952774197;
    cx_ = 320.5;
    cy_ = 240.5;
    lx_ = 640;
    ly_ = 480;
}

Camera::Camera(double f_x, double f_y, double c_x, double c_y)
{
    fx_ = f_x;
    fy_ = f_y;
    cx_ = c_x;
    cy_ = c_y;
    lx_ = 640;
    ly_ = 480;
}

Camera::~Camera(){}

void Camera::setParameters(double f_x, double f_y, double c_x, double c_y)
{
    fx_ = f_x;
    fy_ = f_y;
    cx_ = c_x;
    cy_ = c_y;
}

double Camera::fx(){return fx_;}
double Camera::fy(){return fy_;}
double Camera::cx(){return cx_;}
double Camera::cy(){return cy_;}
double Camera::lx(){return lx_;}
double Camera::ly(){return ly_;}