#ifndef MAV_H
#define MAV_H
#pragma once

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include "Camera.h"

using namespace std;

class MAV
{
private:
    ros::NodeHandle nh;
    double roll;
    double pitch;
    double yaw;

    geometry_msgs::PoseStamped pose_current;
    geometry_msgs::TwistStamped vel_current;
    geometry_msgs::Vector3 acc_current;
    sensor_msgs::Imu imu_current;
    mavros_msgs::State state;

    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber mav_state_sub;

    Camera cam;

public:
    MAV();
    MAV(ros::NodeHandle &nh);
    MAV(ros::NodeHandle &nh, string vehicle, int ID);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void mav_state_cb(const mavros_msgs::State::ConstPtr& msg);
    
    sensor_msgs::Imu getImu();
    geometry_msgs::PoseStamped getPose();
    geometry_msgs::TwistStamped getVel();
    geometry_msgs::Vector3 getAcc();
    mavros_msgs::State getState();
    void setPose(geometry_msgs::Pose Pose);
    void setTwist(geometry_msgs::Twist Twist);
    void setOrientation(geometry_msgs::Quaternion q);

    void setCamera(Camera cam);
    Camera getCamera();

    string topic;
    int id;
    bool pose_init;
    bool vel_init;
    bool imu_init;
    int topic_count;
};

struct MAV_eigen
{
	Eigen::Vector3d r;
    Eigen::Vector3d r_c;
	Eigen::Vector3d v;
	Eigen::Vector3d a_imu;
	Eigen::Vector3d omega_c;
	Eigen::Matrix3d R_w2b;
    Eigen::Quaterniond q;
};

/*=================================================================================================================================
    Conversions
=================================================================================================================================*/

MAV_eigen mavMsg2Eigen(MAV Mav);
std::vector<MAV_eigen> mavsMsg2Eigen(std::vector<MAV> Mavs);
std::vector<MAV_eigen> mavsMsg2Eigen(MAV* Mavs, int mavNum);

#endif