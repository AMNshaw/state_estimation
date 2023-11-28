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
#include <state_estimation/RMSE.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include "Mav.h"
#include "EIF.h"
#include "TEIF.h"
#include "REIF.h"


class Data_process
{
private:
	ros::Subscriber bboxes_sub;
	ros::Subscriber targetPose_sub;
	ros::Subscriber targetVel_sub;
	ros::Subscriber fusedPair_sub;

	std::string bbox_topic;
	std::string targetPose_topic;
	std::string targetVel_topic;
	std::string targetPose_EIF_topic;
	std::string targetVel_EIF_topic;
	std::string targetPose_EIF_err_topic;
	std::string EIFpairs_topic;
	std::string RMSE_topic;

	int state_size;
public:
	Data_process(ros::NodeHandle &nh, string vehicle, int ID, MAV* mavs);
	~Data_process();

	ros::Publisher targetPose_EIF_pub;
	ros::Publisher targetVel_EIF_pub;
	ros::Publisher targetPose_EIF_err_pub;
	ros::Publisher EIFpairs_pub;
	ros::Publisher RMSE_pub;

	std_msgs::Header sync_header;
	state_estimation::EIFpairStamped fusedPair;
	Eigen::MatrixXf fusedOmega;
	Eigen::VectorXf fusedXi;

	Eigen::VectorXf targetState_GT;
	std::vector<float> bboxes;
	bool gotBbox;
	bool gotFusedPair;

	void bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void fusedPair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);

	void set_topic(std::string group_ns, int id);
	void ros2Eigen(MAV_eigen* Mavs_eigen, int mavNum);
	void compare(Eigen::VectorXf X_t);

	MAV* Mavs;
	
	Eigen::Vector3f bboxes_eigen;
};

Data_process::Data_process(ros::NodeHandle &nh, string vehicle, int ID, MAV* mavs)
{
	set_topic(vehicle, ID);
	Mavs = mavs;
	
	gotBbox  = false;

	bboxes_sub = nh.subscribe<std_msgs::Float32MultiArray>(bbox_topic, 2, &Data_process::bboxes_cb, this);
	targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(targetPose_topic, 2, &Data_process::targetPose_cb, this);
	targetVel_sub = nh.subscribe<geometry_msgs::TwistStamped>(targetVel_topic, 2, &Data_process::targetVel_cb, this);
	fusedPair_sub = nh.subscribe<state_estimation::EIFpairStamped>("/HEIF/fusedPair", 2, &Data_process::fusedPair_cb, this);

	targetPose_EIF_pub = nh.advertise<geometry_msgs::PoseStamped>(targetPose_EIF_topic, 1);
	targetVel_EIF_pub = nh.advertise<geometry_msgs::TwistStamped>(targetVel_EIF_topic, 1);
	targetPose_EIF_err_pub = nh.advertise<geometry_msgs::PoseStamped>(targetPose_EIF_err_topic, 1);
	RMSE_pub = nh.advertise<state_estimation::RMSE>(RMSE_topic, 1);

	EIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(EIFpairs_topic, 1);

	targetState_GT.resize(6);
	gotFusedPair = false;

	//fusedXi.setZero(stateSize);
	//fusedOmega.setIdentity(state_size, state_size);
}

Data_process::~Data_process(){};

void Data_process::set_topic(std::string vehicle, int id)
{
	bbox_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/synchronizer/yolov7/boundingBox");
	targetPose_EIF_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/EIF/pose");
	targetVel_EIF_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/EIF/vel");
	targetPose_EIF_err_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/EIF/pose_err");
	EIFpairs_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/EIF/fusionPairs");
	RMSE_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/EIF/RMSE");

	targetPose_topic = std::string("/target/mavros/local_position/pose_initialized");
	targetVel_topic = std::string("/target/mavros/local_position/velocity_local");
}

void Data_process::ros2Eigen(MAV_eigen* Mavs_eigen, int mavNum)
{
	for(int i = 0; i < mavNum; i++)
	{
		Mavs_eigen[i].r(0) = Mavs[i].getPose().pose.position.x;
		Mavs_eigen[i].r(1) = Mavs[i].getPose().pose.position.y;
		Mavs_eigen[i].r(2) = Mavs[i].getPose().pose.position.z;
		Mavs_eigen[i].v(0) = Mavs[i].getVel().twist.linear.x;
		Mavs_eigen[i].v(1) = Mavs[i].getVel().twist.linear.y;
		Mavs_eigen[i].v(2) = Mavs[i].getVel().twist.linear.z;
	}
	Mavs_eigen[MAV::self_pointer].omega_c(0) = Mavs[MAV::self_pointer].getVel().twist.angular.x;
	Mavs_eigen[MAV::self_pointer].omega_c(1) = Mavs[MAV::self_pointer].getVel().twist.angular.y;
	Mavs_eigen[MAV::self_pointer].omega_c(2) = Mavs[MAV::self_pointer].getVel().twist.angular.z;
	Mavs_eigen[MAV::self_pointer].R_w2b = Eigen::Quaternionf(
		Mavs[MAV::self_pointer].getPose().pose.orientation.w,
		Mavs[MAV::self_pointer].getPose().pose.orientation.x,
		Mavs[MAV::self_pointer].getPose().pose.orientation.y,
		Mavs[MAV::self_pointer].getPose().pose.orientation.z
	).toRotationMatrix().inverse();
	bboxes_eigen << bboxes[0], bboxes[1], bboxes[2]; 
	
}

void Data_process::bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if(!gotBbox)
		gotBbox = true;
	bboxes = msg->data;
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

/*
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
*/

void Data_process::fusedPair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	/*
	if(!gotFusedPair)
		gotFusedPair = true;
	fusedPair = *msg;
	Eigen::Map<Eigen::MatrixXf> fusedInfoMat(fusedPair.fusedInfoMat.data(), state_size, state_size);
	fusedOmega = fusedInfoMat.cast<float>();
	Eigen::Map<Eigen::VectorXf> fusedInfoVec(fusedPair.fusedInfoVec.data(), state_size);
	fusedXi = fusedInfoVec.cast<float>();
	*/
}

void Data_process::compare(Eigen::VectorXf X_t)
{
	Eigen::VectorXf E = targetState_GT - X_t;
	Eigen::VectorXf E_p(3), E_v(3);
	state_estimation::RMSE RMSE_data;
	E_p << E(0), E(1), E(2);
	E_v << E(3), E(4), E(5);

	std::cout << "X_t: \n" << X_t << "\n\n";
	std::cout << "RMS_p: " << E_p.norm() << "\nRMS_v: " << E_v.norm() << "\n\n";

	RMSE_data.header = sync_header;
	RMSE_data.RMSE_p = E_p.norm();
	RMSE_data.RMSE_v = E_v.norm();
	RMSE_pub.publish(RMSE_data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

    std::string vehicle;
	bool est_target_acc = 0;
    bool consensus = false;
    int target_state_size = 6;
    int target_measurement_size = 6;
	int mavNum = 3;
    int hz;
	int ID = 0;
    ros::param::get("vehicle", vehicle);
	ros::param::get("ID", ID);
    ros::param::get("rate", hz);
	double last_t;
	double dt;
	MAV::self_pointer = ID-1;

	ros::Rate rate(hz);

	Eigen::VectorXf targetState_EIF;
	geometry_msgs::PoseStamped targetPose_EIF;
	geometry_msgs::TwistStamped targetVel_EIF;
	geometry_msgs::PoseStamped targetPose_EIF_err;
	state_estimation::EIFpairStamped EIFpairs;
	
	MAV Mavs[] = {MAV(nh, vehicle, 1),
				MAV(nh, vehicle, 2),
				MAV(nh, vehicle, 3)};
	mavNum = sizeof(Mavs)/sizeof(Mavs[0]);
	MAV_eigen Mavs_eigen[mavNum];

	Data_process dp(nh, vehicle, ID, Mavs);
	
	while(ros::ok())
	{
		int topics_count = 0;
		for(int i = 0; i < mavNum; i++)
		{
			if(dp.Mavs[i].pose_init)
				topics_count ++;
			else
				printf("[%s_%i]: Waiting for Pose topic...\n", vehicle.c_str(), i+1);
			if(dp.Mavs[i].vel_init)
				topics_count ++;
			else
				printf("[%s_%i]: Waiting for Vel topic...\n", vehicle.c_str(), i+1);
			if(dp.Mavs[i].imu_init)
				topics_count ++;
			else
				printf("[%s_%i]: Waiting for Imu topic...\n", vehicle.c_str(), i+1);
		}

		if(topics_count == 9)
		{
			if(!dp.gotBbox)
				printf("[%s_%i]: Waiting for Boundingbox topic...\n", vehicle.c_str(), ID);
			else
				break;
		}
		
		rate.sleep();
		ros::spinOnce();
	}
	printf("\n[%s_%i EIF]: Topics all checked, start calculating EIF\n", vehicle.c_str(), ID);

	target_EIF Teif(MAV::self_pointer, mavNum, est_target_acc);
	robots_EIF Reif(MAV::self_pointer, mavNum);
	//printf("\n[%s EIF]: EIF constructed\n\n", vehicle.c_str());

	
	last_t = ros::Time::now().toSec();
	dt = ros::Time::now().toSec() - last_t;
    last_t = ros::Time::now().toSec();

	

	
    while(ros::ok())
    {
    	dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();

		dp.ros2Eigen(Mavs_eigen, mavNum);
		Reif.setData(Mavs_eigen);
		Reif.computePredPairs(dt);
		Reif.computeCorrPairs();
		
    	// eif.process(dt, measurement, dp.fusedOmega, dp.fusedXi, dp.gotFusedPair);
    	// EIFpairs.header = dp.sync_header;
    	// EIFpairs.id = ID;
    	// EIFpairs.stateSize = state_size;
    	// getFusionPairs(eif, EIFpairs);
    	// if(consensus == true)
		// {
		// 	dp.EIFpairs_pub.publish(EIFpairs);
		// }
    	// else
    	// 	dp.compare(eif.getTargetState());
    	
		
		rate.sleep();
    	ros::spinOnce();
    }
	
	return 0;
}