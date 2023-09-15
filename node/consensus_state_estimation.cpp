#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <state_estimation/EIFpairStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "EIF.h"
#include "HEIF.h"

using namespace message_filters;
using namespace std;


class Data_process
{
private:
	ros::Publisher fusedPair_pub;

	std::vector<string> fusionPair_topics;
	std::string targetPose_input_topic;
	std::string targetVel_input_topic;

	state_estimation::EIFpairStamped* fusionPairs;
	Eigen::VectorXf targetState_GT;
	Eigen::VectorXf targetState_HEIF;

	Eigen::MatrixXf fusedInfoMat;
	Eigen::VectorXf fusedInfoVec;

	int fusionNum;
	int state_size;
	HEIF heif;
public:
	Data_process(ros::NodeHandle& nh, int num, int stateSize);
	~Data_process();

	void set_topic();

	void fusionPair_sync_cb2(const state_estimation::EIFpairStamped::ConstPtr& ori_pair1,
							const state_estimation::EIFpairStamped::ConstPtr& ori_pair2,
							const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose,
							const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel);

	void fusionPair_sync_cb3(const state_estimation::EIFpairStamped::ConstPtr& ori_pair1,
							const state_estimation::EIFpairStamped::ConstPtr& ori_pair2,
							const state_estimation::EIFpairStamped::ConstPtr& ori_pair3,
							const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose,
							const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel);

};


Data_process::Data_process(ros::NodeHandle& nh, int num, int stateSize) : heif(num, stateSize)
{
	fusionNum = num;
	state_size = stateSize;

	set_topic();
	fusionPairs = new state_estimation::EIFpairStamped[fusionNum];

	fusedPair_pub = nh.advertise<state_estimation::EIFpairStamped>("/HEIF/fusedPair", 1);

	targetState_GT.setZero(state_size);

	if(fusionNum == 2)
	{
		message_filters::Subscriber<state_estimation::EIFpairStamped> fusionPairs_sub1(nh, fusionPair_topics[0], 1);
		message_filters::Subscriber<state_estimation::EIFpairStamped> fusionPairs_sub2(nh, fusionPair_topics[1], 1);
		message_filters::Subscriber<geometry_msgs::PoseStamped> targetPose_sub(nh, targetPose_input_topic, 1);
		message_filters::Subscriber<geometry_msgs::TwistStamped> targetVel_sub(nh, targetVel_input_topic, 1);

		
		typedef message_filters::sync_policies::ApproximateTime<state_estimation::EIFpairStamped,
																state_estimation::EIFpairStamped,
																geometry_msgs::PoseStamped,
																geometry_msgs::TwistStamped>MySyncPolicy;

		
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100),
														 fusionPairs_sub1,
														 fusionPairs_sub2,
														 targetPose_sub,
														 targetVel_sub);

		sync.registerCallback(boost::bind(&Data_process::fusionPair_sync_cb2, this, _1, _2, _3, _4));

		printf("[FusionPair synchronizer]: Start synchronizing all fusion pairs.\n");

		ros::spin();
	}
	else if(fusionNum == 3)
	{
		message_filters::Subscriber<state_estimation::EIFpairStamped> fusionPairs_sub1(nh, fusionPair_topics[0], 1);
		message_filters::Subscriber<state_estimation::EIFpairStamped> fusionPairs_sub2(nh, fusionPair_topics[1], 1);
		message_filters::Subscriber<state_estimation::EIFpairStamped> fusionPairs_sub3(nh, fusionPair_topics[2], 1);
		message_filters::Subscriber<geometry_msgs::PoseStamped> targetPose_sub(nh, targetPose_input_topic, 1);
		message_filters::Subscriber<geometry_msgs::TwistStamped> targetVel_sub(nh, targetVel_input_topic, 1);

		
		typedef message_filters::sync_policies::ApproximateTime<state_estimation::EIFpairStamped,
																state_estimation::EIFpairStamped,
																state_estimation::EIFpairStamped,
																geometry_msgs::PoseStamped,
																geometry_msgs::TwistStamped>MySyncPolicy;

		
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100),
														 fusionPairs_sub1,
														 fusionPairs_sub2,
														 fusionPairs_sub3,
														 targetPose_sub,
														 targetVel_sub);

		sync.registerCallback(boost::bind(&Data_process::fusionPair_sync_cb3, this, _1, _2, _3, _4, _5));

		printf("[FusionPair synchronizer]: Start synchronizing all fusion pairs.\n");

		ros::spin();
	}

	
	
	
}
Data_process::~Data_process(){}

void Data_process::set_topic()
{
	for(int i = 0; i < fusionNum; i++)
		fusionPair_topics.push_back(std::string("/iris_") + to_string(i+1) + std::string("/EIF/fusionPairs"));
	targetPose_input_topic = string("/target/synchronizer/local_position/pose_initialized");
  	targetVel_input_topic = string("/target/synchronizer/local_position/velocity_local");
}

void Data_process::fusionPair_sync_cb2(const state_estimation::EIFpairStamped::ConstPtr& ori_pair1,
									  const state_estimation::EIFpairStamped::ConstPtr& ori_pair2,
									  const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose,
									  const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel)
{
	fusionPairs[0] = *ori_pair1;
	fusionPairs[1] = *ori_pair2;
	
	targetState_GT << ori_targetPose->pose.position.x,
					  ori_targetPose->pose.position.y,
				      ori_targetPose->pose.position.z,
				      ori_targetVel->twist.linear.x,
				      ori_targetVel->twist.linear.y,
					  ori_targetVel->twist.linear.z;

	heif.inputFusionPairs(fusionPairs);
	heif.CI();
	heif.compare(targetState_GT);
	fusedPair_pub.publish(heif.getFusedPairs());
}


void Data_process::fusionPair_sync_cb3(const state_estimation::EIFpairStamped::ConstPtr& ori_pair1,
									  const state_estimation::EIFpairStamped::ConstPtr& ori_pair2,
									  const state_estimation::EIFpairStamped::ConstPtr& ori_pair3,
									  const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose,
									  const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel)
{
	fusionPairs[0] = *ori_pair1;
	fusionPairs[1] = *ori_pair2;
	fusionPairs[2] = *ori_pair3;
	
	targetState_GT << ori_targetPose->pose.position.x,
					  ori_targetPose->pose.position.y,
				      ori_targetPose->pose.position.z,
				      ori_targetVel->twist.linear.x,
				      ori_targetVel->twist.linear.y,
					  ori_targetVel->twist.linear.z;

	heif.inputFusionPairs(fusionPairs);
	heif.CI();
	heif.compare(targetState_GT);
	fusedPair_pub.publish(heif.getFusedPairs());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "msg_synchronizer");
	ros::NodeHandle nh;

	int fusionNum = 2;
	int state_size = 6;

	Data_process dp(nh, fusionNum, state_size);

	return 0;
}