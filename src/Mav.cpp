#include "Mav.h"

int MAV::self_index = 0;

MAV::MAV(){}

MAV::MAV(ros::NodeHandle &nh_)
{
    nh = nh_;
}

MAV::MAV(ros::NodeHandle &nh_, string vehicle, int ID)
{
    nh = nh_;
    pose_init = vel_init = imu_init = false;
    id = ID;
    roll = pitch = yaw = 0;
    topic_count = 0;
    pose_hz = 30.0;

    string prefix = string("/") + vehicle + string("_") + to_string(ID);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(prefix + string("/mavros/local_position/pose_initialized"), 10, &MAV::pose_cb, this);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(prefix + string("/mavros/local_position/velocity_local"), 10, &MAV::vel_cb, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>(prefix + string("/mavros/imu/data"), 10, &MAV::imu_cb, this);
    
}

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_init)
    {
        pose_init = true;
        pose_current = *msg;
    }
    
    if(pose_hz != 0)
        topic_count++;
    if(topic_count >= 30/pose_hz && pose_hz != 0)
    {
        pose_current = *msg;
        topic_count = 0;
    }
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
    tf::Quaternion Q(
        pose_current.pose.orientation.x,
        pose_current.pose.orientation.y,
        pose_current.pose.orientation.z,
        pose_current.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
    tf::Vector3 acc(imu_current.linear_acceleration.x, imu_current.linear_acceleration.y, imu_current.linear_acceleration.z);
    acc_current.x = tf::quatRotate(Q, acc).getX();
    acc_current.y = tf::quatRotate(Q, acc).getY();
    acc_current.z = tf::quatRotate(Q, acc).getZ() - 9.81;
}

geometry_msgs::PoseStamped MAV::getPose(){return pose_current;}
geometry_msgs::TwistStamped MAV::getVel(){return vel_current;}
geometry_msgs::Vector3 MAV::getAcc(){return acc_current;}
double MAV::getYaw(){return yaw;}
void MAV::setPose_hz(float hz){pose_hz = hz;}

void MAV::setPose(geometry_msgs::Pose Pose)
{
    pose_current.header.stamp = ros::Time::now();

    pose_current.pose = Pose;
    tf::Quaternion Q(
        pose_current.pose.orientation.x,
        pose_current.pose.orientation.y,
        pose_current.pose.orientation.z,
        pose_current.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
    
}
void MAV::setTwist(geometry_msgs::Twist Twist)
{
    vel_current.header.stamp = ros::Time::now();

    vel_current.twist = Twist;
}

void MAV::getRPY(float& R, float& P, float& Y)
{
    R = static_cast<float>(roll);
    P = static_cast<float>(pitch);
    Y = static_cast<float>(yaw);
}