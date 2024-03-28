#include <string>
#include <sstream>
#include <vector>
#include <cctype>
#include <cmath>
#include <random>
#include <algorithm>

#include <Eigen/Dense>
#include <ros/ros.h>
#include "ros/param.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/Plot.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>

#include "Mav.h"
#include "TEIF.h"
#include "HEIF_self.h"
#include "HEIF_target.h"
#include "SEIF_pose.h"
#include "SEIF_neighbors.h"
#include "GT_measurement_ros.h"
#include "EIFpairs_ros.h"
#include "Camera.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

	ros::Publisher mavros_fusionPose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
	ros::Publisher mavros_fusionTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/vision_pose/twist", 10);
	ros::Publisher target_fusionPose_pub = nh.advertise<geometry_msgs::PoseStamped>("THEIF/pose", 10);
	ros::Publisher target_fusionTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("THEIF/twist", 10);
	ros::Publisher isTargetEst_pub = nh.advertise<std_msgs::Bool>("THEIF/isTargetEst", 10);

    std::string vehicle;
    bool consensus = false;
	bool position_estimation = false;
	int mavNum = 3;
    int rosRate = 50;
	int ID = 0;
	int state_size = 6;
	double targetTimeTol = 0.05;
	double last_t;
	double dt;
    ros::param::get("vehicle", vehicle);
	ros::param::get("ID", ID);
    ros::param::get("rate", rosRate);
	ros::param::get("consensus", consensus);
	ros::param::get("stateSize", state_size);
	ros::param::get("targetTimeTolerance", targetTimeTol);
	ros::param::get("pos_est", position_estimation);
	
	ros::Rate rate(rosRate);

	geometry_msgs::PoseStamped self_fusedPoseMsg;
	geometry_msgs::TwistStamped self_fusedTwistMsg;
	geometry_msgs::PoseStamped target_fusedPoseMsg;
	geometry_msgs::TwistStamped target_fusedTwistMsg;

	MAV mav(nh);
	EIFpairs_ros eif_ros(nh, vehicle, ID, mavNum);
	Camera cam(nh, true);
	GT_measurement gt_m(nh, ID, 4);
	gt_m.setRosRate(rosRate);
	MAV_eigen mav_eigen;

	

	while(ros::ok())
	{
		if(mav.imu_init)
				break;
		else
			printf("[%s_%i]: Waiting for Imu topic...\n", vehicle.c_str(), ID);
		rate.sleep();
		ros::spinOnce();
	}
	printf("\n[%s_%i EIF]: Topic checked\n", vehicle.c_str(), ID);
	for(int i=0; i< 20; i++)
	{
		rate.sleep();
		ros::spinOnce();
	}
	
	Self_pose_EIF SEIF_pose;
	Self_rel_EIF SEIF_neighbors;
	target_EIF teif(6);
	HEIF_self sheif(6);
	HEIF_target theif(6);

	printf("\n[%s_%i EIF]: EIF constructed\n\n", vehicle.c_str(), ID);

	SEIF_pose.setCurrState(gt_m.getGTs_eigen()[ID]);
	if(position_estimation)
	{
		Eigen::MatrixXd Q(6, 6);
		Q.block(0, 0, 3, 3) = 1e-3*Eigen::MatrixXd::Identity(3, 3); // position
    	Q.block(3, 3, 3, 3) = 8e-2*Eigen::MatrixXd::Identity(3, 3); // velocity
		SEIF_pose.set_process_noise(Q);
	}
	
	dt = 0.001;
	last_t = ros::Time::now().toSec();

	std_msgs::Bool isTargetEst_msg;

    while(ros::ok())
    {
		mav.setOrientation(gt_m.getGTorientation(ID));
		mav_eigen = mavMsg2Eigen(mav);

		/*=================================================================================================================================
			Prediction
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		SEIF_pose.setMavSelfData(mav_eigen);
		if(position_estimation)
			SEIF_pose.setMeasurement(gt_m.getPositionMeasurement());
		SEIF_pose.computePredPairs(dt);
		eif_ros.selfPredEIFpairs_pub.publish(eigen2EifMsg(SEIF_pose.getEIFData(), ID));
		
		SEIF_neighbors.setMavSelfData(mav_eigen);
		SEIF_neighbors.setEIFpredData(SEIF_pose.getEIFData());
		SEIF_neighbors.setLidarMeasurements(gt_m.getLidarMeasurements());
		SEIF_neighbors.setNeighborData(eif_ros.get_curr_fusing_data(eif_ros.neighborsEIFpairs, 0.05));

		// -------------------------------------Target-------------------------------------
		gt_m.bbox_check();
		if(gt_m.ifCameraMeasure())
		{
			if(!teif.filter_init)
				teif.setInitialState(gt_m.getBboxEigen());
			teif.setCamera(cam);
			teif.setMavSelfData(mav_eigen); 
			teif.setMeasurement(gt_m.getBboxEigen());
			teif.setSEIFpredData(SEIF_pose.getEIFData());
		 	teif.computePredPairs(dt);
		}

		/*=================================================================================================================================
			Correction
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		SEIF_pose.computeCorrPairs();
		SEIF_neighbors.computeCorrPairs();

		// -------------------------------------Target-------------------------------------
		if(gt_m.ifCameraMeasure())
		{
		 	teif.computeCorrPairs();
			eif_ros.self2TgtEIFpairs_pub.publish(eigen2EifMsg(teif.getTgtData(), ID));
		}

		/*=================================================================================================================================
			Fusion
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		sheif.setSelfEstData(SEIF_pose.getEIFData());
		sheif.setNeighborEstData(SEIF_neighbors.getEIFData());
		sheif.process();
		SEIF_pose.setFusionPairs(sheif.getFusedCov(), sheif.getFusedState());
		
		// std::cout << "SEIF:\n";
		// eif_ros.selfState_Plot_pub.publish(compare(gt_m.getGTs_eigen()[ID], sheif.getFusedState()));
		
		// -------------------------------------Target-------------------------------------
		std::vector<EIF_data> allTgtEIFData;
		allTgtEIFData = eif_ros.get_curr_fusing_data(eif_ros.rbs2Tgt_EIFPairs, 0.05);
		if(gt_m.ifCameraMeasure())
			allTgtEIFData.push_back(teif.getTgtData());
		theif.setTargetEstData(allTgtEIFData);
		theif.process();
		if(gt_m.ifCameraMeasure())
		{
			teif.setFusionPairs(theif.getFusedCov(), theif.getFusedState(), ros::Time::now().toSec());
			
			// if(theif.QP_init(15, 2))
			// {
			// 	theif.QP_pushData(ros::Time::now().toSec(), theif.getFusedState().segment(0, 3));
			// 	if(theif.computeQP());
			// 		teif.setEstAcc(theif.getQpAcc());
			// }
		}
		std::cout << "TEIF:\n";
		(compare(gt_m.getGTs_eigen()[0], theif.getFusedState()));
		
		
		/*=================================================================================================================================
			Publish to mavros for feedback
		=================================================================================================================================*/
		
		// -------------------------------------Position-------------------------------------
		self_fusedPoseMsg.header.frame_id = "/world";
		self_fusedPoseMsg.header.stamp = ros::Time::now();
		self_fusedPoseMsg.pose.position.x = sheif.getFusedState()(0);
		self_fusedPoseMsg.pose.position.y = sheif.getFusedState()(1);
		self_fusedPoseMsg.pose.position.z = sheif.getFusedState()(2);
		self_fusedPoseMsg.pose.orientation.w = mav_eigen.q.w();
		self_fusedPoseMsg.pose.orientation.x = mav_eigen.q.x();
		self_fusedPoseMsg.pose.orientation.y = mav_eigen.q.y();
		self_fusedPoseMsg.pose.orientation.z = mav_eigen.q.z();

		target_fusedPoseMsg.header.frame_id = "/world";
		target_fusedPoseMsg.header.stamp = ros::Time::now();
		target_fusedPoseMsg.pose.position.x = theif.getFusedState()(0);
		target_fusedPoseMsg.pose.position.y = theif.getFusedState()(1);
		target_fusedPoseMsg.pose.position.z = theif.getFusedState()(2);

		// -------------------------------------Velocity-------------------------------------
		self_fusedTwistMsg.header.stamp = ros::Time::now();
		self_fusedTwistMsg.twist.linear.x = sheif.getFusedState()(3);
		self_fusedTwistMsg.twist.linear.y = sheif.getFusedState()(4);
		self_fusedTwistMsg.twist.linear.z = sheif.getFusedState()(5);
		self_fusedTwistMsg.twist.angular.x = mav_eigen.omega_c(0);
		self_fusedTwistMsg.twist.angular.y = mav_eigen.omega_c(1);
		self_fusedTwistMsg.twist.angular.z = mav_eigen.omega_c(2);

		target_fusedTwistMsg.header.stamp = ros::Time::now();
		target_fusedTwistMsg.twist.linear.x = theif.getFusedState()(3);
		target_fusedTwistMsg.twist.linear.y = theif.getFusedState()(4);
		target_fusedTwistMsg.twist.linear.z = theif.getFusedState()(5);

		// -------------------------------------Camera detect?-------------------------------------
		isTargetEst_msg.data = gt_m.ifCameraMeasure();

		// -------------------------------------Publish-------------------------------------
		mavros_fusionPose_pub.publish(self_fusedPoseMsg);
		mavros_fusionTwist_pub.publish(self_fusedTwistMsg);
		target_fusionPose_pub.publish(target_fusedPoseMsg);
		target_fusionTwist_pub.publish(target_fusedTwistMsg);
		isTargetEst_pub.publish(isTargetEst_msg);

		/*=================================================================================================================================
			Descrete time
		=================================================================================================================================*/
		dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
		
		rate.sleep();
    	ros::spinOnce();
    }
	
	return 0;
}
