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

#include "HEIF.h"

using namespace std;


class Data_process
{
private:
	ros::Subscriber *fusionPair_sub;
	ros::Subscriber targetPose_sub;
	ros::Subscriber targetVel_sub;

	std::vector<string> fusionPair_topics;
	std::string targetPose_input_topic;
	std::string targetVel_input_topic;
	std::string RMSE_topic;

	state_estimation::EIFpairStamped* fusionPairs;
	Eigen::VectorXf targetState_GT;
	std::vector<EIF_data> est_data;

	int fusionNum;
	int state_size;
	double max_time;

	HEIF heif;
public:
	ros::Publisher fusedPair_pub;
	ros::Publisher RMSE_pub;

	bool *topic_check;

	Data_process(ros::NodeHandle& nh, int num, int stateSize);
	~Data_process();

	void set_topic();
	std::vector<EIF_data> get_curr_fuse_data();
	void fusion();

	void fusionPair_cb(const state_estimation::EIFpairStamped::ConstPtr& ori_fusionPairs);
	void targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose);
	void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel);

	state_estimation::EIFpairStamped* getFusionPairs();

	void compare(Eigen::VectorXf fusedX_t);
	EIF_data eifMsg2Eigen(state_estimation::EIFpairStamped eifMsg);

};

Data_process::Data_process(ros::NodeHandle& nh, int num, int stateSize) : heif(stateSize)
{
	fusionNum = num;
	state_size = stateSize;
	
	set_topic();
	fusionPairs = new state_estimation::EIFpairStamped[fusionNum];
	fusionPair_sub = new ros::Subscriber[fusionNum];
	topic_check = new bool[fusionNum];
	for(int i=0; i < fusionNum; i++)
		topic_check[i] = false;

	for(int i=0; i < fusionNum; i++)
		fusionPair_sub[i] = nh.subscribe<state_estimation::EIFpairStamped>(fusionPair_topics[i], 5, &Data_process::fusionPair_cb, this);

	targetPose_sub = nh.subscribe<geometry_msgs::PoseStamped>(targetPose_input_topic, 1, &Data_process::targetPose_cb, this);
	targetVel_sub = nh.subscribe<geometry_msgs::TwistStamped>(targetVel_input_topic, 1, &Data_process::targetVel_cb, this);

	fusedPair_pub = nh.advertise<state_estimation::EIFpairStamped>("/HEIF/fusedPair", 1);
	RMSE_pub = nh.advertise<state_estimation::RMSE>("/HEIF/RMSE", 1);

	targetState_GT.setZero(state_size);	

	std::cout << "HEIF constructed" << "\n";
}
Data_process::~Data_process()
{
	delete[] fusionPairs;
	delete[] fusionPair_sub;
	delete[] topic_check;
}

void Data_process::set_topic()
{
	for(int i = 0; i < fusionNum; i++)
		fusionPair_topics.push_back(std::string("/iris_") + to_string(i+1) + std::string("/TEIF/fusionPairs"));
	targetPose_input_topic = string("/target/mavros/local_position/pose_initialized");
  	targetVel_input_topic = string("/target/mavros/local_position/velocity_local");
}

std::vector<EIF_data> Data_process::get_curr_fuse_data()
{
	est_data.clear();
	max_time = -1;
	for(int i=0; i<fusionNum; i++)
	{
		if(fusionPairs[i].header.stamp.toSec() > max_time)
			max_time = fusionPairs[i].header.stamp.toSec();
	}
	for(int i=0; i<fusionNum; i++)
	{
		if(abs(fusionPairs[i].header.stamp.toSec() - max_time) <= 0.01)
			est_data.push_back(eifMsg2Eigen(fusionPairs[i]));
	}
	return est_data;
}

EIF_data Data_process::eifMsg2Eigen(state_estimation::EIFpairStamped eifMsg)
{
	EIF_data est_object;
	est_object.P_hat = Eigen::Map<Eigen::MatrixXf>(eifMsg.P_hat.data(), state_size, state_size);
	est_object.X_hat = Eigen::Map<Eigen::VectorXf>(eifMsg.X_hat.data(), state_size);
	est_object.s = Eigen::Map<Eigen::MatrixXf>(eifMsg.s.data(), state_size, state_size);
	est_object.y = Eigen::Map<Eigen::VectorXf>(eifMsg.y.data(), state_size);

	return est_object;
}

void Data_process::fusion()
{
	heif.setData(get_curr_fuse_data());
	heif.CI();
	compare(heif.getFusedState());
}


void Data_process::fusionPair_cb(const state_estimation::EIFpairStamped::ConstPtr& ori_fusionPairs)
{
	int index = ori_fusionPairs->id - 1;
	fusionPairs[index] = *ori_fusionPairs;
}


void Data_process::targetPose_cb(const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose)
{
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
	Eigen::VectorXf err = targetState_GT.segment(0, 6) - fusedX_t.segment(0, 6);
	Eigen::Vector3f E_p;
	Eigen::Vector3f E_v;
	state_estimation::RMSE RMSE_data;

	E_p << err(0), err(1), err(2);
	E_v << err(3), err(4), err(5);

	std::cout << "[HEIF]: X_t\n" << fusedX_t << "\n\n";
	std::cout << "[HEIF]: RMS_p: " << E_p.norm() << "\n[HEIF]: RMS_v: " << E_v.norm() << "\n\n";

	RMSE_data.header.stamp = ros::Time::now();
	RMSE_data.RMSE_p = E_p.norm();
	RMSE_data.RMSE_v = E_v.norm();
	RMSE_pub.publish(RMSE_data);
}

state_estimation::EIFpairStamped* Data_process::getFusionPairs(){return fusionPairs;}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "msg_synchronizer");
	ros::NodeHandle nh;
	
	int hz = 200;
	int fusionNum = 2;
	int state_size = 9;
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
		dp.fusion();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}