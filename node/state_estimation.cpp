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
#include "SEIF.h"
#include "HEIF.h"
#include "CommonFuncs.h"


class Data_process
{
private:
	ros::Subscriber bboxes_sub;
	ros::Subscriber targetPose_sub;
	ros::Subscriber targetVel_sub;
	ros::Subscriber tgtFusedPair_sub;
	ros::Subscriber* rbs2SelfEIFpairs_sub;
	ros::Subscriber* rbs2TgtEIFpairs_sub;

	std::string bbox_topic;
	std::string targetPose_topic;
	std::string targetVel_topic;
	std::string* self2RbsEIFpairs_pub_topic;
	std::string* rbs2SelfEIFpairs_sub_topic;
	std::string self2TgtEIFpairs_pub_topic;
	std::string* rbs2TgtEIFpairs_sub_topic;
	std::string tgtStateRMSE_topic;
	std::string selfStateRMSE_topic;

	int state_size;
	int mavNum;
	int self_id;
	int self_index;
public:
	Data_process(ros::NodeHandle &nh, string vehicle, int ID, int mavnum);
	~Data_process();

	ros::Publisher tgtState_RMSE_pub;
	ros::Publisher selfState_RMSE_pub;
	ros::Publisher self2TgtEIFpairs_pub;
	ros::Publisher* self2RbsEIFpairs_pub;

	std_msgs::Header sync_header;
	state_estimation::EIFpairStamped* rbs2Self_EIFPairs;
	state_estimation::EIFpairStamped* rbs2Tgt_EIFPairs;
	std::vector<EIF_data> est_data;

	Eigen::VectorXf targetState_GT;
	std::vector<float> bboxes;
	bool gotBbox;
	bool gotFusedPair;

	void bboxes_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void rbs2SelfEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);
	void rbs2TgtEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg);
	void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

	void set_topic(std::string group_ns, int id);
	Eigen::Vector3f bboxMsg2Eigen();
	std::vector<EIF_data> get_curr_fusing_data(state_estimation::EIFpairStamped* EIFPairs, float tolerance);
	state_estimation::EIFpairStamped* getRbs2SelfEIFmsg();
	state_estimation::EIFpairStamped* getRbs2TgtEIFmsg();

};

Data_process::Data_process(ros::NodeHandle &nh, string vehicle, int ID, int mavnum)
{
	self_id = ID;
	self_index = ID-1;
	mavNum = mavnum;
	set_topic(vehicle, self_id);

	gotBbox  = false;
	gotFusedPair = false;
	rbs2Self_EIFPairs = new state_estimation::EIFpairStamped[mavNum];
	rbs2Tgt_EIFPairs = new state_estimation::EIFpairStamped[mavNum];

	/////////////////////////////////////////////////Subscriber/////////////////////////////////////////////////
	bboxes_sub = nh.subscribe<std_msgs::Float32MultiArray>(bbox_topic, 2, &Data_process::bboxes_cb, this);
	targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(targetPose_topic, 2, &Data_process::targetPose_cb, this);
	targetVel_sub = nh.subscribe<geometry_msgs::TwistStamped>(targetVel_topic, 2, &Data_process::targetVel_cb, this);

	rbs2SelfEIFpairs_sub = new ros::Subscriber[mavNum];
	rbs2TgtEIFpairs_sub = new ros::Subscriber[mavNum];
	for(int i=0; i<mavNum; i++)
		if(i != self_index)
		{
			rbs2SelfEIFpairs_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(rbs2SelfEIFpairs_sub_topic[i], 1, &Data_process::rbs2SelfEIFpair_cb, this);
			rbs2TgtEIFpairs_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(rbs2TgtEIFpairs_sub_topic[i], 1, &Data_process::rbs2TgtEIFpair_cb, this);
		}

	/////////////////////////////////////////////////Publisher /////////////////////////////////////////////////

	tgtState_RMSE_pub = nh.advertise<state_estimation::RMSE>(tgtStateRMSE_topic, 1);
	selfState_RMSE_pub = nh.advertise<state_estimation::RMSE>(selfStateRMSE_topic, 1);
	self2TgtEIFpairs_pub = nh.advertise<state_estimation::EIFpairStamped>(self2TgtEIFpairs_pub_topic, 1);

	self2RbsEIFpairs_pub = new ros::Publisher[mavNum];
	for(int i=0; i<mavNum; i++)
	 	if(i != self_index)
			self2RbsEIFpairs_pub[i] = nh.advertise<state_estimation::EIFpairStamped>(self2RbsEIFpairs_pub_topic[i], 1);	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	targetState_GT.resize(6);
}

Data_process::~Data_process()
{
	delete[] self2RbsEIFpairs_pub_topic;
	delete[] rbs2SelfEIFpairs_sub_topic;
	delete[] rbs2TgtEIFpairs_sub_topic;

	delete[] self2RbsEIFpairs_pub;
	delete[] rbs2SelfEIFpairs_sub;
	delete[] rbs2TgtEIFpairs_sub;

	delete[] rbs2Self_EIFPairs;
	delete[] rbs2Tgt_EIFPairs;
}

void Data_process::set_topic(std::string vehicle, int id)
{
	bbox_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/synchronizer/yolov7/boundingBox");
	self2TgtEIFpairs_pub_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/TEIF/fusionPairs");
	tgtStateRMSE_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/TEIF/RMSE");
	selfStateRMSE_topic = std::string("/") + vehicle + std::string("_") + std::to_string(id) + std::string("/SHEIF/RMSE");

	targetPose_topic = std::string("/target/mavros/local_position/pose_initialized");
	targetVel_topic = std::string("/target/mavros/local_position/velocity_local");

	self2RbsEIFpairs_pub_topic = new std::string[mavNum];
	rbs2SelfEIFpairs_sub_topic = new std::string[mavNum];
	rbs2TgtEIFpairs_sub_topic = new std::string[mavNum];
	for(int i=0; i<mavNum; i++)
		if(i != self_index)
		{
			self2RbsEIFpairs_pub_topic[i] = std::string("/") + vehicle + std::string("_") + std::to_string(id)
			+ std::string("/REIF_") + std::to_string(i+1) + std::string("/fusionPairs");
			rbs2SelfEIFpairs_sub_topic[i] = std::string("/") + vehicle + std::string("_") + std::to_string(i+1)
			+ std::string("/REIF_") + std::to_string(id) + std::string("/fusionPairs");
			rbs2TgtEIFpairs_sub_topic[i] = std::string("/") + vehicle + std::string("_") + std::to_string(i+1)
			+ std::string("/TEIF") + std::string("/fusionPairs");
			std::cout << rbs2TgtEIFpairs_sub_topic[i] << "\n";
		}
}


Eigen::Vector3f Data_process::bboxMsg2Eigen()
{
	Eigen::Vector3f bboxes_eigen;
	bboxes_eigen << bboxes[0], bboxes[1], bboxes[2];
	return bboxes_eigen;
}

std::vector<EIF_data> Data_process::get_curr_fusing_data(state_estimation::EIFpairStamped* EIFPairs, float tolerance)
{
	est_data.clear();
	for(int i=0; i<mavNum; i++)
	{
		if(abs(EIFPairs[i].header.stamp.toSec() - ros::Time::now().toSec()) <= tolerance)
			est_data.push_back(eifMsg2Eigen(EIFPairs[i]));
	}
	return est_data;
}

void Data_process::rbs2SelfEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	int index = msg->id-1;
	rbs2Self_EIFPairs[index] = *msg;
}

void Data_process::rbs2TgtEIFpair_cb(const state_estimation::EIFpairStamped::ConstPtr& msg)
{
	int index = msg->id-1;
	rbs2Tgt_EIFPairs[index] = *msg;
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

state_estimation::EIFpairStamped* Data_process::getRbs2SelfEIFmsg(){return rbs2Self_EIFPairs;}
state_estimation::EIFpairStamped* Data_process::getRbs2TgtEIFmsg(){return rbs2Tgt_EIFPairs;}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

    std::string vehicle;
	bool est_target_acc = false;
    bool consensus = false;
	int mavNum = 3;
    int hz;
	int ID = 0;
	int state_size = 6;
	float targetTimeTol = 0.05;
    ros::param::get("vehicle", vehicle);
	ros::param::get("ID", ID);
    ros::param::get("rate", hz);
	ros::param::get("consensus", consensus);
	ros::param::get("stateSize", state_size);
	ros::param::get("targetTimeTol", targetTimeTol);
	double last_t;
	double dt;
	MAV::self_index = ID-1;

	ros::Rate rate(hz);

	MAV Mavs[] = {MAV(nh, vehicle, 1),
				MAV(nh, vehicle, 2),
				MAV(nh, vehicle, 3)};
	mavNum = sizeof(Mavs)/sizeof(Mavs[0]);
	
	std::vector<MAV_eigen> Mavs_eigen(mavNum);
	Data_process dp(nh, vehicle, ID, mavNum);

	while(ros::ok())
	{
		int topics_count = 0;
		for(int i = 0; i < mavNum; i++)
		{
			if(Mavs[i].pose_init)
				topics_count ++;
			else
				printf("[%s_%i]: Waiting for Pose topic...\n", vehicle.c_str(), i+1);
			if(Mavs[i].vel_init)
				topics_count ++;
			else
				printf("[%s_%i]: Waiting for Vel topic...\n", vehicle.c_str(), i+1);
			if(Mavs[i].imu_init)
				topics_count ++;
			else
				printf("[%s_%i]: Waiting for Imu topic...\n", vehicle.c_str(), i+1);
		}
		if(topics_count == mavNum*3)
			break;
		rate.sleep();
		ros::spinOnce();
	}
	printf("\n[%s_%i EIF]: Topics all checked\n", vehicle.c_str(), ID);
	
	Self_acc_EIF Seif(MAV::self_index, mavNum);
	Mavs_eigen = mavsMsg2Eigen(Mavs, mavNum);
	Seif.setCurrPose(Mavs_eigen[MAV::self_index].r);
	robots_EIF Reif(MAV::self_index, mavNum);
	target_EIF Teif(state_size, MAV::self_index, mavNum);
	HEIF selfState_HEIF(state_size);
	HEIF targetState_HEIF(state_size);

	printf("\n[%s_%i EIF]: EIF constructed\n\n", vehicle.c_str(), ID);
	
    last_t = ros::Time::now().toSec();
	dt = 0.001;
    while(ros::ok())
    {
		Mavs_eigen = mavsMsg2Eigen(Mavs, mavNum);

		//////////////////////////////////// Self_acc_EIF ////////////////////////////////////
		std::cout << "SEIF:\n";
		Seif.setData(Mavs_eigen[MAV::self_index]);
		Seif.computePredPairs(dt);
		Seif.computeCorrPairs();
		selfState_HEIF.setData(dp.get_curr_fusing_data(dp.getRbs2SelfEIFmsg(), 0.03), Seif.getSelfData());
		selfState_HEIF.process();
		Seif.setFusionPairs(selfState_HEIF.getFusedCov(), selfState_HEIF.getFusedState());
		dp.selfState_RMSE_pub.publish(compare(Mavs_eigen[MAV::self_index], Seif.getSelfData().X));
		////////////////////////////////// Other Robots EIF //////////////////////////////////
		Reif.setData(Mavs_eigen);
		Reif.computePredPairs(dt);
		Reif.computeCorrPairs();
		for(int i=0; i<mavNum; i++)
			if(i != MAV::self_index)
				dp.self2RbsEIFpairs_pub[i].publish(eigen2EifMsg(Reif.getRbsData()[i], ID));
		//dp.selfState_RMSE_pub.publish(compare(Mavs_eigen[1], Reif.getRbsData()[1].X));

		//////////////////////////////////// Target_EIF /////////////////////////////////////
		if(dp.gotBbox)
		{
			std::cout << "TEIF:\n";
			std::vector<EIF_data> allTgtEIFData;
			Teif.setData(Mavs_eigen[MAV::self_index], dp.bboxMsg2Eigen());
			Teif.computePredPairs(dt, Reif.getRbsData());
			Teif.computeCorrPairs();
			dp.self2TgtEIFpairs_pub.publish(eigen2EifMsg(Teif.getTgtData(), ID));
			allTgtEIFData = dp.get_curr_fusing_data(dp.getRbs2TgtEIFmsg(), targetTimeTol);
			allTgtEIFData.push_back(Teif.getTgtData());
			targetState_HEIF.setData(allTgtEIFData);
			targetState_HEIF.process();
			Teif.setFusionPairs(targetState_HEIF.getFusedCov(), targetState_HEIF.getFusedState());
			//dp.tgtState_RMSE_pub.publish(compare(dp.targetState_GT, Teif.getTgtData().X));
			dp.tgtState_RMSE_pub.publish(compare(dp.targetState_GT, targetState_HEIF.getFusedState()));
		}

		/////////////////////////////////////////////////////////////////////////////////////
    	dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
		
		rate.sleep();
    	ros::spinOnce();
    }
	
	return 0;
}