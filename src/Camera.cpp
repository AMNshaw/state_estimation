#include "Camera.h"

Camera::Camera()
{
    fx_ = 565.6008952774197;
    fy_ = 565.6008952774197;
    cx_ = 320.5;
    cy_ = 240.5;
    lx_ = 640;
    ly_ = 480;

	t_b2c<< 0.1, 0, 0; 
	R_b2c<< 0, -1, 0,
			0, 0, -1,
			1, 0, 0;
}

Camera::Camera(ros::NodeHandle &nh_, bool gimbal)
{

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

Eigen::Vector3d Camera::t_B2C(){return t_b2c;}
Eigen::Matrix3d Camera::R_B2C(){return R_b2c;}