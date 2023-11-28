#include "Mav.h"

int MAV::self_pointer = 0;

MAV::MAV(){}

MAV::MAV(ros::NodeHandle &nh_)
{
    nh = nh_;
}

MAV::MAV(ros::NodeHandle &nh_, string vehicle, int ID)
{
    nh = nh_;
    pose_init = vel_init = imu_init = false;
    string prefix = string("/") + vehicle + string("_") + to_string(ID);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(prefix + string("/mavros/local_position/pose_initialized"), 10, &MAV::pose_cb, this);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(prefix + string("/mavros/local_position/velocity_local"), 10, &MAV::vel_cb, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>(prefix + string("/mavros/imu/data"), 10, &MAV::imu_cb, this);
    id = ID;
    roll = pitch = yaw = 0;
}

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_init)
        pose_init = true;
    pose_current = *msg;

    tf::Quaternion Q(
        pose_current.pose.orientation.x,
        pose_current.pose.orientation.y,
        pose_current.pose.orientation.z,
        pose_current.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}

void MAV::vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if(!vel_init)
        vel_init = true;
    vel_current = *msg;
}

void MAV::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(!imu_init)
        imu_init = true;
    imu_current = *msg;
}

geometry_msgs::PoseStamped MAV::getPose(){return pose_current;}
geometry_msgs::TwistStamped MAV::getVel(){return vel_current;}
double MAV::getYaw(){return yaw;}