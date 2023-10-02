#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "ros/param.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <state_estimation/EIFpairStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include "EIF.h"


class Data_process
{
private:
	ros::Subscriber bboxes_sub;
	ros::Subscriber self_pose_sub;
	ros::Subscriber self_vel_sub;
	ros::Subscriber targetPose_sub;
	ros::Subscriber targetVel_sub;
	ros::Subscriber fusedPair_sub;
	ros::Subscriber self_imu_sub;

	std::string bbox_topic;
	std::string self_pose_topic;
	std::string self_vel_topic;
	std::string targetPose_topic;
	std::string targetVel_topic;
	std::string targetPose_EIF_topic;
	std::string targetVel_EIF_topic;
	std::string targetPose_EIF_err_topic;
	std::string EIFpairs_topic;
	std::string self_imu_topic;

	int state_size;
public:
	Data_process(ros::NodeHandle &nh, std::string group_ns, int stateSize);
	~Data_process();

	ros::Publisher targetPose_EIF_pub;
	ros::Publisher targetVel_EIF_pub;
	ros::Publisher targetPose_EIF_err_pub;
	ros::Publisher EIFpairs_pub;

	geometry_msgs::PoseStamped self_pose;
	geometry_msgs::TwistStamped self_vel;
	Eigen::VectorXd self_acc;
	std_msgs::Header sync_header;
	state_estimation::EIFpairStamped fusedPair;
	Eigen::MatrixXd fusedOmega;
	Eigen::VectorXd fusedXi;

	Eigen::VectorXd targetState_GT;
	std::vector<float> bboxes;
	bool gotBbox;
	bool gotSelfPose;
	bool gotSelfVel;
	bool gotSelfImu;
	bool gotFusedPair;

	void bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void self_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void self_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void self_imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
	void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void fusedPair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);

	void set_topic(std::string group_ns);

};

Data_process::Data_process(ros::NodeHandle &nh, std::string group_ns, int stateSize)
{
	gotBbox = gotSelfPose = gotSelfVel = gotSelfImu = false;
	set_topic(group_ns);
	state_size = stateSize;

	bboxes_sub = nh.subscribe<std_msgs::Float32MultiArray>(bbox_topic, 2, &Data_process::bboxes_cb, this);
	self_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(self_pose_topic, 2, &Data_process::self_pose_cb, this);
	self_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(self_vel_topic, 2, &Data_process::self_vel_cb, this);
	targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(targetPose_topic, 2, &Data_process::targetPose_cb, this);
	targetVel_sub = nh.subscribe<geometry_msgs::TwistStamped>(targetVel_topic, 2, &Data_process::targetVel_cb, this);
	fusedPair_sub = nh.subscribe<state_estimation::EIFpairStamped>("/HEIF/fusedPair", 2, &Data_process::fusedPair_cb, this);
	self_imu_sub = nh.subscribe<sensor_msgs::Imu>(self_imu_topic, 2, &Data_process::self_imu_cb, this);

	targetPose_EIF_pub = nh.advertise<geometry_msgs::PoseStamped>(targetPose_EIF_topic, 1);
	targetVel_EIF_pub = nh.advertise<geometry_msgs::TwistStamped>(targetVel_EIF_topic, 1);
	targetPose_EIF_err_pub = nh.advertise<geometry_msgs::PoseStamped>(targetPose_EIF_err_topic, 1);

	EIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(EIFpairs_topic, 1);

	targetState_GT.resize(state_size);
	gotFusedPair = false;

	fusedXi.setZero(stateSize);
	fusedOmega.setIdentity(state_size, state_size);
}

Data_process::~Data_process(){};

void Data_process::set_topic(std::string group_ns)
{
	bbox_topic = std::string("/") + group_ns + std::string("/synchronizer/yolov7/boundingBox");
	self_pose_topic = std::string("/") + group_ns + std::string("/synchronizer/local_position/pose_initialized");
	self_vel_topic = std::string("/") + group_ns + std::string("/synchronizer/local_position/velocity_local");
	targetPose_topic = std::string("/target/synchronizer/local_position/pose_initialized");
	targetVel_topic = std::string("/target/synchronizer/local_position/velocity_local");
	targetPose_EIF_topic = std::string("/") + group_ns + std::string("/EIF/pose");
	targetVel_EIF_topic = std::string("/") + group_ns + std::string("/EIF/vel");
	targetPose_EIF_err_topic = std::string("/") + group_ns + std::string("/EIF/pose_err");
	EIFpairs_topic = std::string("/") + group_ns + std::string("/EIF/fusionPairs");
	self_imu_topic = std::string("/") + group_ns + std::string("/synchronizer/imu/data");
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
	sync_header = msg->header;
	self_pose = *msg;
}

void Data_process::self_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	if(!gotSelfVel)
		gotSelfVel = true;
	self_vel = *msg;
}

void Data_process::self_imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(!gotSelfImu)
		gotSelfImu = true;

	Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	Eigen::VectorXd acc_b(3);
	acc_b << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	self_acc = q*acc_b;
	self_acc(2) = self_acc(2) - 9.81;

	//std::cout << "self_acc:\n" << self_acc << std::endl;
}

void Data_process::targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	targetState_GT(0) = msg->pose.position.x;
	targetState_GT(1) = msg->pose.position.y;
	targetState_GT(2) = msg->pose.position.z;
}

void Data_process::targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	targetState_GT(3) = msg->twist.linear.x;
	targetState_GT(4) = msg->twist.linear.y;
	targetState_GT(5) = msg->twist.linear.z;
}

void getFusionPairs(EIF eif, state_estimation::EIFpairStamped& EIFpairs)
{
	Eigen::MatrixXd predInfoMat, corrInfoMat;
	Eigen::VectorXd predInfoVec, corrInfoVec;

	eif.getPredictionPairs(&predInfoMat, &predInfoVec);
	eif.getCorrectionPairs(&corrInfoMat, &corrInfoVec);

	std::vector<float> predInfoMat_vec(predInfoMat.data(), predInfoMat.data() + predInfoMat.size());
	std::vector<float> predInfoVec_vec(predInfoVec.data(), predInfoVec.data() + predInfoVec.size());
	std::vector<float> corrInfoMat_vec(corrInfoMat.data(), corrInfoMat.data() + corrInfoMat.size());
	std::vector<float> corrInfoVec_vec(corrInfoVec.data(), corrInfoVec.data() + corrInfoVec.size());

	EIFpairs.predInfoMat = predInfoMat_vec;
	EIFpairs.predInfoVec = predInfoVec_vec;
	EIFpairs.corrInfoMat = corrInfoMat_vec;
	EIFpairs.corrInfoVec = corrInfoVec_vec;
}

void Data_process::fusedPair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	if(!gotFusedPair)
		gotFusedPair = true;
	fusedPair = *msg;
	Eigen::Map<Eigen::MatrixXf> fusedInfoMat(fusedPair.fusedInfoMat.data(), state_size, state_size);
	fusedOmega = fusedInfoMat.cast<double>();
	Eigen::Map<Eigen::VectorXf> fusedInfoVec(fusedPair.fusedInfoVec.data(), state_size);
	fusedXi = fusedInfoVec.cast<double>();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

    std::string vehicle;
    bool consensus;
    int state_size = 6;
    int measurement_size = 3;
    int hz;
    ros::param::get("vehicle", vehicle);
    ros::param::get("consensus", consensus);
    ros::param::get("stateSize", state_size);
    ros::param::get("rate", hz);
;
	ros::Rate rate(hz);

	double last_t;
	double dt;

	Eigen::VectorXd targetState_EIF;
	geometry_msgs::PoseStamped targetPose_EIF;
	geometry_msgs::TwistStamped targetVel_EIF;
	geometry_msgs::PoseStamped targetPose_EIF_err;
	state_estimation::EIFpairStamped EIFpairs;

	Eigen::VectorXd measurement;
	measurement.resize(measurement_size);

	Data_process dp(nh, vehicle, state_size);
	printf("[%s EIF]: Fusion state size: %i \n", vehicle.c_str(), state_size);
	printf("[%s EIF]: Fusion measurement size: %i \n\n", vehicle.c_str(), measurement_size);

	while(ros::ok() && (!dp.gotBbox || !dp.gotSelfPose || !dp.gotSelfVel || !dp.gotSelfImu))
	{
		//printf("[%s]: Waiting for topics...\n", vehicle.c_str());
		ros::spinOnce();
	}
	printf("\n[%s EIF]: Topics all checked, start calculating EIF\n\n", vehicle.c_str());

	EIF eif(state_size, measurement_size, consensus);
	printf("\n[%s EIF]: EIF constructed\n\n", vehicle.c_str());

	last_t = ros::Time::now().toSec();

    while(ros::ok())
    {
    	dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();

    	if(state_size == 6)
    		eif.setSelfState(dp.self_pose, dp.self_vel);
    	else if(state_size == 9)
    		eif.setSelfState(dp.self_pose, dp.self_vel, dp.self_acc);


    	measurement << dp.bboxes[0], dp.bboxes[1], dp.bboxes[2];
    	eif.process(dt, measurement, dp.fusedOmega, dp.fusedXi, dp.gotFusedPair);
    	
    	EIFpairs.header = dp.sync_header;
    	EIFpairs.stateSize = state_size;
    	getFusionPairs(eif, EIFpairs);
    	dp.EIFpairs_pub.publish(EIFpairs);

    	eif.compare(dp.targetState_GT);
    	/*
    	targetState_EIF = eif.getTargetState();
    	targetPose_EIF_err.header = dp.sync_header;
    	targetPose_EIF_err.pose.position.x = dp.targetState_GT(0) - targetState_EIF(0);
    	targetPose_EIF_err.pose.position.y = dp.targetState_GT(1) - targetState_EIF(1);
    	targetPose_EIF_err.pose.position.z = dp.targetState_GT(2) - targetState_EIF(2);
    	dp.targetPose_EIF_err_pub.publish(targetPose_EIF_err);
    	*/


    	ros::spinOnce();
    }
}