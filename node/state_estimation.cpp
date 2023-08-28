#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "ros/param.h"
#include <std_msgs/Float32MultiArray.h>

#include "EIF.h"


class Data_process
{
private:
	ros::Subscriber bboxes_sub;
	ros::Subscriber self_pose_sub;
	ros::Subscriber self_vel_sub;
	ros::Subscriber targetPose_sub;
	ros::Subscriber targetVel_sub;

	std::string bbox_topic;
	std::string self_pose_topic;
	std::string self_vel_topic;
	std::string targetPose_topic;
	std::string targetVel_topic;

public:
	Data_process(ros::NodeHandle &nh, std::string group_ns);
	~Data_process();

	geometry_msgs::PoseStamped self_pose;
	geometry_msgs::TwistStamped self_vel;
	Eigen::VectorXd targetState;
	std::vector<float> bboxes;
	bool gotBbox;
	bool gotSelfPose;
	bool gotSelfVel;

	void bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void self_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void self_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

	void set_topic(std::string group_ns);

};

Data_process::Data_process(ros::NodeHandle &nh, std::string group_ns)
{
	gotBbox = gotSelfPose = gotSelfVel = false;
	set_topic(group_ns);
	bboxes_sub = nh.subscribe<std_msgs::Float32MultiArray>(bbox_topic, 2, &Data_process::bboxes_cb, this);
	self_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(self_pose_topic, 2, &Data_process::self_pose_cb, this);
	self_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(self_vel_topic, 2, &Data_process::self_vel_cb, this);
	targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(targetPose_topic, 2, &Data_process::targetPose_cb, this);
	targetVel_sub = nh.subscribe<geometry_msgs::TwistStamped>(targetVel_topic, 2, &Data_process::targetVel_cb, this);

	targetState.resize(6);
}

Data_process::~Data_process(){};

void Data_process::set_topic(std::string group_ns)
{
	bbox_topic = std::string("/") + group_ns + std::string("/synchronizer/yolov7/boundingBox");
	self_pose_topic = std::string("/") + group_ns + std::string("/synchronizer/local_position/pose_initialized");
	self_vel_topic = std::string("/") + group_ns + std::string("/synchronizer/local_position/velocity_local");
	targetPose_topic = std::string("/target/synchronizer/local_position/pose_initialized");
	targetVel_topic = std::string("/target/synchronizer/local_position/velocity_local");
}

void Data_process::bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if(!gotBbox)
		gotBbox = true;
	bboxes = msg->data;
}

void Data_process::self_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(!gotSelfPose)
		gotSelfPose = true;
	self_pose = *msg;
}

void Data_process::self_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	if(!gotSelfVel)
		gotSelfVel = true;
	self_vel = *msg;
}

void Data_process::targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	targetState(0) = msg->pose.position.x;
	targetState(1) = msg->pose.position.y;
	targetState(2) = msg->pose.position.z;
}

void Data_process::targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	targetState(3) = msg->twist.linear.x;
	targetState(4) = msg->twist.linear.y;
	targetState(5) = msg->twist.linear.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

    std::string vehicle;
    ros::param::get("vehicle", vehicle);

	ros::Rate rate(100);

	int state_size = 6;
	int measurement_size = 3;

	double last_t;
	double dt;

	Eigen::VectorXd measurement;
	measurement.resize(3);

	Data_process dp(nh, vehicle);

	while(ros::ok() && (!dp.gotBbox || !dp.gotSelfPose || !dp.gotSelfVel))
	{
		std::cout << "Waiting for topics..." << std::endl;
		ros::spinOnce();
	}
	std::cout << "Topics all checked" << std::endl;

	EIF eif(state_size, measurement_size);

	last_t = ros::Time::now().toSec();

    while(ros::ok())
    {
    	dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
    	eif.setSelfState(dp.self_pose, dp.self_vel);
    	eif.predict(dt);
    	measurement << dp.bboxes[0], dp.bboxes[1], dp.bboxes[2];
    	if(measurement(2) != 18.0)
    		eif.correct(measurement);
    	eif.compare(dp.targetState);

    	//std::cout << "ground truth:\n" << dp.targetState << "\n\n";

    	ros::spinOnce();
    }
}