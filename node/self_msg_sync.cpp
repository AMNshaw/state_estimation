#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include "ros/param.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <state_estimation/Int32MultiArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace message_filters;

class Image_process
{
private:
  ros::Publisher sync_yolo_pub;
  ros::Publisher sync_depth_pub;
  ros::Publisher sync_bbox_pub;
  ros::Publisher sync_selfPose_pub;
  ros::Publisher sync_selfVel_pub;
  ros::Publisher sync_targetPose_pub;
  ros::Publisher sync_targetVel_pub;

  sensor_msgs::Image sync_img_yolo;
  sensor_msgs::Image sync_img_depth;
  std_msgs::Float32MultiArray sync_bbox_msgs;
  geometry_msgs::PoseStamped sync_selfPose;
  geometry_msgs::TwistStamped sync_selfVel;
  geometry_msgs::PoseStamped sync_targetPose;
  geometry_msgs::TwistStamped sync_targetVel;

  string vehicle;
  string yolo_input_topic;
  string yolo_output_topic;
  string depth_input_topic;
  string depth_output_topic;
  string bbox_input_topic;
  string bbox_output_topic;
  string self_pose_input_topic;
  string self_pose_output_topic;
  string self_vel_input_topic;
  string self_vel_output_topic;
  string targetPose_input_topic;
  string targetPose_output_topic;
  string targetVel_input_topic;
  string targetVel_output_topic;
  

  int bbox_col;
  bool start;

  void sync_cb(const sensor_msgs::ImageConstPtr& ori_yolo,
               const sensor_msgs::ImageConstPtr& ori_depth,
               const state_estimation::Int32MultiArrayStamped::ConstPtr& ori_bbox,
               const geometry_msgs::PoseStamped::ConstPtr& ori_selfPose,
               const geometry_msgs::TwistStamped::ConstPtr& ori_selfVel,
               const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose,
               const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel);

public:
  Image_process(ros::NodeHandle &nh, string group_ns);
  ~Image_process();

  float getDepth(int u, int v);
  void set_topic(string group_ns);
  void set_bbox_col(int col);
  void reArrangeBbox(state_estimation::Int32MultiArrayStamped bbox_msgs);
};

Image_process::Image_process(ros::NodeHandle &nh, string group_ns)
{
  start = false;
  vehicle = group_ns;
  set_topic(group_ns);

  sync_yolo_pub = nh.advertise<sensor_msgs::Image>(yolo_output_topic, 1);
  sync_depth_pub = nh.advertise<sensor_msgs::Image>(depth_output_topic, 1);
  sync_bbox_pub = nh.advertise<std_msgs::Float32MultiArray>(bbox_output_topic, 1);
  sync_selfPose_pub = nh.advertise<geometry_msgs::PoseStamped>(self_pose_output_topic, 1);
  sync_selfVel_pub = nh.advertise<geometry_msgs::TwistStamped>(self_vel_output_topic, 1);
  sync_targetPose_pub = nh.advertise<geometry_msgs::PoseStamped>(targetPose_output_topic, 1);
  sync_targetVel_pub = nh.advertise<geometry_msgs::TwistStamped>(targetVel_output_topic, 1);


  message_filters::Subscriber<sensor_msgs::Image> img_yolo_sub(nh, yolo_input_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> img_depth_sub(nh, depth_input_topic, 1);
  message_filters::Subscriber<state_estimation::Int32MultiArrayStamped> bbox_msg_sub(nh, bbox_input_topic, 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> self_pose_sub(nh, self_pose_input_topic, 1);
  message_filters::Subscriber<geometry_msgs::TwistStamped> self_vel_sub(nh, self_vel_input_topic, 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> target_pose_sub(nh, targetPose_input_topic, 1);
  message_filters::Subscriber<geometry_msgs::TwistStamped> target_vel_sub(nh, targetVel_input_topic, 1);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          state_estimation::Int32MultiArrayStamped,
                                                          geometry_msgs::PoseStamped,
                                                          geometry_msgs::TwistStamped,
                                                          geometry_msgs::PoseStamped,
                                                          geometry_msgs::TwistStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), 
                                                   img_yolo_sub,
                                                   img_depth_sub,
                                                   bbox_msg_sub,
                                                   self_pose_sub,
                                                   self_vel_sub,
                                                   target_pose_sub,
                                                   target_vel_sub);
  sync.registerCallback(boost::bind(&Image_process::sync_cb, this, _1, _2, _3, _4, _5, _6, _7));

  bbox_col = 5;

  ros::spin();
}

Image_process::~Image_process(){}

void Image_process::sync_cb(const sensor_msgs::ImageConstPtr& ori_yolo, 
                            const sensor_msgs::ImageConstPtr& ori_depth,
                            const state_estimation::Int32MultiArrayStamped::ConstPtr& ori_bbox,
                            const geometry_msgs::PoseStamped::ConstPtr& ori_selfPose,
                            const geometry_msgs::TwistStamped::ConstPtr& ori_selfVel,
                            const geometry_msgs::PoseStamped::ConstPtr& ori_targetPose,
                            const geometry_msgs::TwistStamped::ConstPtr& ori_targetVel)
{
  if(!start)
  {
    start = true;
    cout << "[" << vehicle << " Message_synchronizer]: Start synchronizing messages... \n\n";
  }

  sync_img_yolo = *ori_yolo;
  sync_img_depth = *ori_depth;
  reArrangeBbox(*ori_bbox);
  sync_selfPose = *ori_selfPose;
  sync_selfVel = *ori_selfVel;
  sync_targetPose = *ori_targetPose;
  sync_targetVel = *ori_targetVel;

  /*
  ROS_INFO("%s image_yolo stamp value is: %f", vehicle.c_str(), sync_img_yolo.header.stamp.toSec());
  ROS_INFO("%s image_depth stamp value is: %f", vehicle.c_str(), sync_img_depth.header.stamp.toSec());
  ROS_INFO("%s bbox_msg stamp value is: %f", vehicle.c_str(), sync_bbox_msgs.header.stamp.toSec());
  */

  sync_yolo_pub.publish(sync_img_yolo);
  sync_depth_pub.publish(sync_img_depth);
  sync_bbox_pub.publish(sync_bbox_msgs);
  sync_selfPose_pub.publish(sync_selfPose);
  sync_selfVel_pub.publish(sync_selfVel);
  sync_targetPose_pub.publish(sync_targetPose);
  sync_targetVel_pub.publish(sync_targetVel);
}

void Image_process::reArrangeBbox(state_estimation::Int32MultiArrayStamped bbox_msgs)
{
  float u, v;
  vector<int> bbox_data = bbox_msgs.data;
  vector<float> reArrangeBbox_data;

  if(bbox_data.size() > 0)
  {
    for(int i = 0; i <bbox_data.size(); i+=bbox_col)
    {
      u = (float)(bbox_data[i+1] + bbox_data[i+3])/2;
      v = (float)(bbox_data[i+2] + bbox_data[i+4])/2;
      reArrangeBbox_data.push_back(u);
      reArrangeBbox_data.push_back(v);
      reArrangeBbox_data.push_back(getDepth((int)u, (int)v));
    }
    sync_bbox_msgs.data = reArrangeBbox_data;
  }
}

float Image_process::getDepth(int u, int v)
{
  float depth = 0;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(sync_img_depth, sensor_msgs::image_encodings::TYPE_32FC1);
  
  depth += cv_ptr->image.at<float>(v, u);
  depth += cv_ptr->image.at<float>(v+1, u);
  depth += cv_ptr->image.at<float>(v-1, u);
  depth += cv_ptr->image.at<float>(v, u+1);
  depth += cv_ptr->image.at<float>(v, u-1);
  depth /=5;

  return depth;
}

void Image_process::set_topic(string group_ns)
{

  string prefix = string("/")+group_ns;

  yolo_input_topic = prefix + string("/yolov7/yolov7/visualization");
  depth_input_topic = prefix + string("/camera/depth/image_raw");
  bbox_input_topic = prefix + string("/yolov7/yolov7/boundingBox");
  self_pose_input_topic = prefix + string("/mavros/local_position/pose_initialized");
  self_vel_input_topic = prefix + string("/mavros/local_position/velocity_local");
  targetPose_input_topic = string("/target/mavros/local_position/pose_initialized");
  targetVel_input_topic = string("/target/mavros/local_position/velocity_local");

  cout << "[" << group_ns << " Message_synchronizer]: Input topic was set:\n"
                          << yolo_input_topic << endl 
                          << depth_input_topic << endl
                          << bbox_input_topic << endl
                          << self_pose_input_topic << endl
                          << self_vel_input_topic << endl
                          <<targetPose_input_topic << endl;

  yolo_output_topic = prefix + string("/synchronizer/yolov7/visualization");
  depth_output_topic = prefix + string("/synchronizer/camera/depth/image_raw");
  bbox_output_topic = prefix + string("/synchronizer/yolov7/boundingBox");
  self_pose_output_topic = prefix + string("/synchronizer/local_position/pose_initialized");
  self_vel_output_topic = prefix + string("/synchronizer/local_position/velocity_local");
  targetPose_output_topic = string("/target/synchronizer/local_position/pose_initialized");
  targetVel_output_topic = string("/target/synchronizer/local_position/velocity_local");

  cout << "[" << group_ns << " Message_synchronizer]: Output topic was set:\n"
                          << yolo_output_topic << endl 
                          << depth_output_topic << endl
                          << bbox_output_topic << endl
                          << self_pose_output_topic << endl
                          << self_vel_output_topic << endl
                          << targetVel_output_topic << "\n\n";
}

void Image_process::set_bbox_col(int col){bbox_col = col;}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_synchronizer");
  ros::NodeHandle nh;

  string group_ns;
  ros::param::get("vehicle", group_ns);

  Image_process process(nh, group_ns);
  
  

  return 0;
}
