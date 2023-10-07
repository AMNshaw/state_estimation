#include <string>
#include <sstream>
#include <vector>
#include <deque>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/RMSE.h>

#include "EIF.h"
#include "HEIF.h"

using namespace std;


class Data_process
{
private:
	ros::Publisher fusedPair_pub;
	ros::Publisher RMSE_pub;

	ros::Subscriber *fusionPair_sub;
	ros::Subscriber targetPose_sub;
	ros::Subscriber targetVel_sub;

	std::vector<string> fusionPair_topics;
	std::string targetPose_input_topic;
	std::string targetVel_input_topic;
	std::string RMSE_topic;

	state_estimation::EIFpairStamped* fusionPairs;
	deque<state_estimation::EIFpairStamped>* fusionPairsQue;
	Eigen::VectorXf targetState_GT;

	Eigen::MatrixXf fusedInfoMat;
	Eigen::VectorXf fusedInfoVec;

	std_msgs::Header sync_header;

	int fusionNum;
	int state_size;
	double* time;
	double min_time;
	

	HEIF heif;
public:

	bool *topic_check;

	Data_process(ros::NodeHandle& nh, int num, int stateSize);
	~Data_process();

	void set_topic();
	void sync_process();
	void fusion();

	void fusionPair_sync_cb(const state_estimation::EIFpairStamped::ConstPtr& ori_fusionPairs);
	void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose);
	void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel);

	void compare(Eigen::VectorXf fusedX_t);

};


Data_process::Data_process(ros::NodeHandle& nh, int num, int stateSize) : heif(num, stateSize)
{
	fusionNum = num;
	state_size = stateSize;

	set_topic();
	fusionPairs = new state_estimation::EIFpairStamped[fusionNum];
	fusionPairsQue = new deque<state_estimation::EIFpairStamped>[fusionNum];
	fusionPair_sub = new ros::Subscriber[fusionNum];
	time = new double[fusionNum];
	topic_check = new bool[fusionNum];
	for(int i=0; i < fusionNum; i++)
		topic_check[i] = false;

	for(int i = 0; i < fusionNum; i++)
		fusionPair_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(fusionPair_topics[i], 10, &Data_process::fusionPair_sync_cb, this);
	targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(targetPose_input_topic, 1, &Data_process::targetPose_cb, this);
	targetVel_sub = nh.subscribe<geometry_msgs::TwistStamped>(targetVel_input_topic, 1, &Data_process::targetVel_cb, this);

	fusedPair_pub = nh.advertise<state_estimation::EIFpairStamped>("/HEIF/fusedPair", 10);
	RMSE_pub = nh.advertise<state_estimation::RMSE>("/HEIF/RMSE", 10);

	targetState_GT.setZero(state_size);	

	std::cout << "HEIF constructed" << "\n";
}
Data_process::~Data_process()
{
	delete[] fusionPairs;
	delete[] fusionPair_sub;
	delete[] fusionPairsQue;
	delete[] time;
	delete[] topic_check;
}

void Data_process::set_topic()
{
	for(int i = 0; i < fusionNum; i++)
		fusionPair_topics.push_back(std::string("/iris_") + to_string(i+1) + std::string("/EIF/fusionPairs"));
	targetPose_input_topic = string("/target/synchronizer/local_position/pose_initialized");
  	targetVel_input_topic = string("/target/synchronizer/local_position/velocity_local");
}

void Data_process::fusionPair_sync_cb(const state_estimation::EIFpairStamped::ConstPtr& ori_fusionPairs)
{
	int id = ori_fusionPairs->id - 1;
	if(!topic_check[id])
		topic_check[id] = true;

	fusionPairsQue[id].push_front(*ori_fusionPairs);
	/*if(fusionPairsQue[id].size() > 100)
		fusionPairsQue[id].pop_back()*/;
	time[id] = fusionPairsQue[id].front().header.stamp.toSec();
	min_time = time[0];
	for(int i = 0; i < fusionNum; i++)
	{
		if(time[i] < min_time)
			min_time = time[i];
	}
}

void Data_process::sync_process()
{
	//std::cout<< "min_time: " << min_time << "\n\n";

	for(int i =0; i < fusionNum; i++)
	{
		//cout << "time: " << fusionPairsQue[i].front().header.stamp.toSec() << "\npop one" << endl;
		
		while(time[i] - min_time >= 0.003)
		{
			time[i] = fusionPairsQue[i].front().header.stamp.toSec();
			if(fusionPairsQue[i].size() > 1)
				fusionPairsQue[i].pop_front();
		}
		
		fusionPairs[i] = fusionPairsQue[i].front();
	}
}

void Data_process::fusion()
{
	heif.inputFusionPairs(fusionPairs);
	heif.CI();
	fusedPair_pub.publish(heif.getFusedPairs());
	compare(heif.getTargetState());
}

void Data_process::targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose)
{
	sync_header = ori_targetPose->header;
	targetState_GT(0) = ori_targetPose->pose.position.x;
	targetState_GT(1) = ori_targetPose->pose.position.y;
	targetState_GT(2) = ori_targetPose->pose.position.z;
}

void Data_process::targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel)
{
	targetState_GT(3) = ori_targetVel->twist.linear.x;
	targetState_GT(4) = ori_targetVel->twist.linear.y;
	targetState_GT(5) = ori_targetVel->twist.linear.z;
}

void Data_process::compare(Eigen::VectorXf fusedX_t)
{
	Eigen::VectorXf err = targetState_GT - fusedX_t;
	Eigen::VectorXf E_p(state_size/2);
	Eigen::VectorXf E_v(state_size/2);
	state_estimation::RMSE RMSE_data;

	E_p << err(0), err(1), err(2);
	E_v << err(3), err(4), err(5);

	std::cout << "[HEIF]: X_t\n" << fusedX_t << "\n\n";
	std::cout << "[HEIF]: RMS_p: " << E_p.norm() << "\n[HEIF]: RMS_v: " << E_v.norm() << "\n\n";

	RMSE_data.header = sync_header;
	RMSE_data.RMSE_p = E_p.norm();
	RMSE_data.RMSE_v = E_v.norm();
	RMSE_pub.publish(RMSE_data);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "msg_synchronizer");
	ros::NodeHandle nh;
	
	int hz = 100;
	int fusionNum = 2;
	int state_size = 6;
	ros::param::get("stateSize", state_size);
	ros::param::get("fusionNum", fusionNum);
	ros::param::get("rate", hz);

	Data_process dp(nh, fusionNum, state_size);

	ros::Rate rate(hz);
	while(ros::ok())
	{
		bool topic_all_check = true;
		for(int i = 0; i< fusionNum; i++)
			if(dp.topic_check[i] == false)
				topic_all_check == false;
		if(topic_all_check)
			break;
		ros::spinOnce();
		rate.sleep();
	}

	for(int i = 0; i < 100; i++)
	{
		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok())
	{
		dp.sync_process();
		dp.fusion();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}