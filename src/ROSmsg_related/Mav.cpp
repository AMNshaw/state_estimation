#include "Mav.h"

MAV::MAV(){}

MAV::MAV(ros::NodeHandle &nh_)
{
    nh = nh_;

    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10, &MAV::pose_cb, this);
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/vision_pose/twist", 10, &MAV::vel_cb, this);
    imu_sub = nh_.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &MAV::imu_cb, this);
    mav_state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &MAV::mav_state_cb, this);
}

MAV::MAV(ros::NodeHandle &nh_, string vehicle, int ID)
{
    nh = nh_;
    pose_init = vel_init = imu_init = false;
    id = ID;
    roll = pitch = yaw = 0;
    topic_count = 0;

    string prefix = string("/") + vehicle + string("_") + to_string(ID);
    if(id != 0)
    {
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(prefix + string("/mavros/vision_pose/pose"), 10, &MAV::pose_cb, this);
        vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>(prefix + string("/mavros/vision_pose/twist"), 10, &MAV::vel_cb, this);
        imu_sub = nh_.subscribe<sensor_msgs::Imu>(prefix + string("/mavros/imu/data"), 10, &MAV::imu_cb, this);
    }
    else
    {
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/leader/formation/pose", 10, &MAV::pose_cb, this);
        vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/leader/formation/velocity", 10, &MAV::vel_cb, this);
    }
    mav_state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &MAV::mav_state_cb, this);
}

void MAV::mav_state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
    state = *msg;
}

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_init)
        pose_init = true;
    pose_current = *msg;
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
    pose_current.pose.orientation = imu_current.orientation;
    vel_current.twist.angular = imu_current.angular_velocity;
    acc_current = imu_current.linear_acceleration;
}

sensor_msgs::Imu MAV::getImu(){return imu_current;}
geometry_msgs::PoseStamped MAV::getPose(){return pose_current;}
geometry_msgs::TwistStamped MAV::getVel(){return vel_current;}
geometry_msgs::Vector3 MAV::getAcc(){return acc_current;}
mavros_msgs::State MAV::getState(){return state;}

void MAV::setPose(geometry_msgs::Pose Pose)
{
    pose_current.header.stamp = ros::Time::now();
    pose_current.pose = Pose;
}
void MAV::setTwist(geometry_msgs::Twist Twist)
{
    vel_current.header.stamp = ros::Time::now();
    vel_current.twist = Twist;
}

void MAV::setOrientation(geometry_msgs::Quaternion q)
{
    pose_current.pose.orientation = q;
}

/*=================================================================================================================================
    Conversions
=================================================================================================================================*/

MAV_eigen mavMsg2Eigen(MAV Mav)
{
	MAV_eigen Mav_eigen;

	Mav_eigen.r(0) = Mav.getPose().pose.position.x;
	Mav_eigen.r(1) = Mav.getPose().pose.position.y;
	Mav_eigen.r(2) = Mav.getPose().pose.position.z;
	Mav_eigen.v(0) = Mav.getVel().twist.linear.x;
	Mav_eigen.v(1) = Mav.getVel().twist.linear.y;
	Mav_eigen.v(2) = Mav.getVel().twist.linear.z;
	Mav_eigen.a_imu(0) = Mav.getAcc().x;
	Mav_eigen.a_imu(1) = Mav.getAcc().y;
	Mav_eigen.a_imu(2) = Mav.getAcc().z;
	
	Mav_eigen.omega_c(0) = Mav.getVel().twist.angular.x;
	Mav_eigen.omega_c(1) = Mav.getVel().twist.angular.y;
	Mav_eigen.omega_c(2) = Mav.getVel().twist.angular.z;
	Mav_eigen.R_w2b = Eigen::Quaterniond(
		Mav.getPose().pose.orientation.w,
		Mav.getPose().pose.orientation.x,
		Mav.getPose().pose.orientation.y,
		Mav.getPose().pose.orientation.z
	).toRotationMatrix().inverse();
	Mav_eigen.q.w() = Mav.getPose().pose.orientation.w;
	Mav_eigen.q.x() = Mav.getPose().pose.orientation.x;
	Mav_eigen.q.y() = Mav.getPose().pose.orientation.y;
	Mav_eigen.q.z() = Mav.getPose().pose.orientation.z;

	return Mav_eigen;
}

std::vector<MAV_eigen> mavsMsg2Eigen(std::vector<MAV> Mavs)
{
    std::vector<MAV_eigen> Mavs_eigen(Mavs.size());
	for(int i=0; i< Mavs.size(); i++)
		Mavs_eigen[i] = mavMsg2Eigen(Mavs[i]);

    return Mavs_eigen;
}

std::vector<MAV_eigen> mavsMsg2Eigen(MAV* Mavs, int mavNum)
{
    std::vector<MAV_eigen> Mavs_eigen;
	for(int i=0; i< mavNum; i++)
		Mavs_eigen.push_back(mavMsg2Eigen(Mavs[i]));

    return Mavs_eigen;
}