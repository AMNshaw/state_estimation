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
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>

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
    sensor_msgs::Imu imu_current;

    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber imu_sub;

public:
    MAV();
    MAV(ros::NodeHandle &nh);
    MAV(ros::NodeHandle &nh, string vehicle, int ID);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void mav_state_cb(const mavros_msgs::State::ConstPtr& msg);
    geometry_msgs::PoseStamped getPose();
    geometry_msgs::TwistStamped getVel();
    mavros_msgs::State getCurrentState();
    double getYaw();

    string topic;
    int id;
    bool pose_init;
    bool vel_init;
    bool imu_init;

    static int self_index;
};

#endif