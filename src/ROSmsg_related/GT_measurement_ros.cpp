#include "GT_measurement_ros.h"

GT_measurement::GT_measurement(ros::NodeHandle& nh_, int id, int mavnum)
{
    nh = nh_;
    ID = id;
    self_index = ID-1;
    mavNum = mavnum;
    formation_num = mavNum-1;

	/*=================================================================================================================================
		groundtruth
	=================================================================================================================================*/
  	groundTruth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 30, &GT_measurement::groundTruth_cb, this);
	GTs_rate = 500;
	GTs_count = 0;
	GTs = new MAV[mavNum];

	/*=================================================================================================================================
        Lidar, position
    ===============================================================================================================================*/
	lidar_rate = 10;
	position_rate = 10;

	/*=================================================================================================================================
        Camera boundingBox
    =================================================================================================================================*/	
    bboxes_sub = nh.subscribe<std_msgs::Float64MultiArray>("synchronizer/yolov7/boundingBox", 2, &GT_measurement::bboxes_cb, this);
	bbox_count = 0;
	no_bbox_count = 0;
	checkCount = 0;
	bbox_eigen_past << 320, 240, 4;
	bbox_eigen = bbox_eigen_past;
	gotBbox  = false;
}

GT_measurement::~GT_measurement()
{
    delete[] GTs;
}

void GT_measurement::setRosRate(int rate)
{
	rosRate = rate;
}

/*=================================================================================================================================
    groundtruth
=================================================================================================================================*/

void GT_measurement::groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	GTs_count++; // GroundTruth call back rate = 500hz

	////////////////////////// get groundTruth model states and arrange their ID////////////////////
	std::vector<string> name = msg->name;
	for(int i=0; i< name.size(); i++)
	{
		if(std::isdigit(name[i].back())) ////// First one is ground, skip it
		{
			GTs[int(name[i].back()-'0')].setPose(msg->pose[i]);
			GTs[int(name[i].back()-'0')].setTwist(msg->twist[i]);
		}
	}
	GTs_eigen = mavsMsg2Eigen(GTs, mavNum);
	std::vector<MAV_eigen> formation_eigen_GT(GTs_eigen.begin()+1, GTs_eigen.begin()+GTs_eigen.size()); // First one is target, we want all UAV

	////////////////////////// Transform from groundtruth to measurements,  ////////////////////////
	static std::default_random_engine generator;
	if(GTs_count % (GTs_rate/lidar_rate) == 0) // lidar_rate = 10hz means that we do a measurement evry 50 count 
		lidarMeasurements = lidarMeasure(formation_eigen_GT, generator);
	if(GTs_count % (GTs_rate/position_rate) == 0) // position_rate = 10hz means that we do a measurement evry 50 count 
		positionMeasurement = positionMeasure(GTs_eigen[ID], generator);
	if(GTs_count == GTs_rate)
		GTs_count = 0;
}

std::vector<MAV_eigen> GT_measurement::getGTs_eigen(){return GTs_eigen;}
geometry_msgs::Quaternion GT_measurement::getGTorientation(int ID){return GTs[ID].getPose().pose.orientation;}


/*=================================================================================================================================
    Lidar, position
===============================================================================================================================*/

std::vector<Eigen::Vector4d> GT_measurement::lidarMeasure(std::vector<MAV_eigen> formation_GT, std::default_random_engine generator)
{
	Eigen::Vector4d measurement;
	std::vector<Eigen::Vector4d> measurements;
	Eigen::Vector3d r_ns_B;
	Eigen::Matrix3d R_W2B_i = formation_GT[self_index].R_w2b.inverse();
	for(int i=0; i<formation_num; i++)
	{
		if(i != self_index)
		{
			r_ns_B = R_W2B_i*(formation_GT[i].r - formation_GT[self_index].r);

			measurement(0) = sqrt(pow(r_ns_B(0), 2) + pow(r_ns_B(1), 2) + pow(r_ns_B(2), 2));
			measurement(1) = acos(r_ns_B(2)/measurement(0)); // theta
			measurement(2) = atan2(r_ns_B(1), r_ns_B(0)); // phi
			measurement(3) = i+1; // ID

			std::normal_distribution<double> n_D(0.0, 0.02);
			std::normal_distribution<double> n_theta(0.0, 0.035);
			std::normal_distribution<double> n_phi(0.0, 0.035);
			measurement(0) += n_D(generator);
			measurement(1) += n_theta(generator);
			measurement(2) += n_phi(generator);

			measurements.push_back(measurement);
		}
	}
	return measurements;
}

Eigen::Vector3d GT_measurement::positionMeasure(MAV_eigen GT_eigen, std::default_random_engine generator)
{
	Eigen::Vector3d measurement = GT_eigen.r;

	std::normal_distribution<double> n_x(0.0, 0.05);
	std::normal_distribution<double> n_y(0.0, 0.05);
	std::normal_distribution<double> n_z(0.0, 0.05);
	measurement(0) += n_x(generator);
	measurement(1) += n_y(generator);
	measurement(2) += n_z(generator);

	return measurement;
}

std::vector<Eigen::Vector4d> GT_measurement::getLidarMeasurements(){return lidarMeasurements;}
Eigen::Vector3d GT_measurement::getPositionMeasurement(){return positionMeasurement;}

/*=================================================================================================================================
    Camera boundingBox
=================================================================================================================================*/
void GT_measurement::bboxes_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    bboxes_raw = msg->data;
	
	std::vector<Eigen::Vector3d> bboxes;
	if(bboxes_raw.size() > 3)
	{
		double min_dist = 99999;
		for(size_t i=0; i<bboxes_raw.size(); i+=3)
		{
			Eigen::Vector3d bbox(bboxes_raw[i], bboxes_raw[i+1], bboxes_raw[i+2]);
			bboxes.push_back(bbox);
		}
		for(auto& bbox : bboxes)
		{
			double dist = sqrt(pow(bbox(0) - bbox_eigen_past(0), 2) + pow(bbox(1) - bbox_eigen_past(1), 2));
			if(dist < min_dist)
			{
				min_dist = dist;
				bbox_eigen = bbox;
			}
		}
	}
	else if(bboxes_raw.size() == 3)
			bbox_eigen << bboxes_raw[0], bboxes_raw[1], bboxes_raw[2];
	no_bbox_count = 0;
}

bool GT_measurement::ifCameraMeasure(){return gotBbox;}
void GT_measurement::bbox_check()
{
	if(!gotBbox)
	{
		no_bbox_count = 0;
		checkCount++;
		if(checkCount == rosRate)
		{
			checkCount = 0;
			bbox_count = 0;
			gotBbox = false;
		}
		if(bbox_eigen != bbox_eigen_past)
		{
			bbox_count++;
			if(bbox_count == 10)
			{
				bbox_count = 0;
				checkCount = 0;
				gotBbox = true;
			}
		}
	}
	else
	{
		if(bbox_eigen == bbox_eigen_past)
		{
			no_bbox_count++;
			if(no_bbox_count == rosRate)
				gotBbox = false;
		}
	}
	bbox_eigen_past = bbox_eigen;
}

Eigen::Vector3d GT_measurement::getBboxEigen(){return bbox_eigen;}