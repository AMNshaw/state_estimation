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

using namespace std;
using namespace message_filters;

class Image_process
{
private:
  ros::Publisher sync_yolo_pub;
  ros::Publisher sync_depth_pub;
  ros::Publisher sync_bbox_pub;

  sensor_msgs::Image sync_img_yolo;
  sensor_msgs::Image sync_img_depth;
  std_msgs::Float32MultiArray sync_bbox_msgs;

  string yolo_input_topic;
  string yolo_output_topic;
  string depth_input_topic;
  string depth_output_topic;
  string bbox_input_topic;
  string bbox_output_topic;
  string vehicle;

  int bbox_col;
  bool start;

  void sync_cb(const sensor_msgs::ImageConstPtr& ori_yolo,
               const sensor_msgs::ImageConstPtr& ori_depth,
               const state_estimation::Int32MultiArrayStamped::ConstPtr& ori_bbox);

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

  message_filters::Subscriber<sensor_msgs::Image> img_yolo_sub(nh, yolo_input_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> img_depth_sub(nh, depth_input_topic, 1);
  message_filters::Subscriber<state_estimation::Int32MultiArrayStamped> bbox_msg_sub(nh, bbox_input_topic, 1);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          state_estimation::Int32MultiArrayStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), 
                                                   img_yolo_sub,
                                                   img_depth_sub,
                                                   bbox_msg_sub);
  sync.registerCallback(boost::bind(&Image_process::sync_cb, this, _1, _2, _3));

  bbox_col = 5;

  ros::spin();
}

Image_process::~Image_process(){}

void Image_process::sync_cb(const sensor_msgs::ImageConstPtr& ori_yolo, 
                            const sensor_msgs::ImageConstPtr& ori_depth,
                            const state_estimation::Int32MultiArrayStamped::ConstPtr& ori_bbox)
{
  if(!start)
  {
    start = true;
    cout << "[" << vehicle << " Message_synchronizer]: Start synchronizing messages... \n\n";
  }

  sync_img_yolo = *ori_yolo;
  sync_img_depth = *ori_depth;
  reArrangeBbox(*ori_bbox);

  /*
  ROS_INFO("%s image_yolo stamp value is: %f", vehicle.c_str(), sync_img_yolo.header.stamp.toSec());
  ROS_INFO("%s image_depth stamp value is: %f", vehicle.c_str(), sync_img_depth.header.stamp.toSec());
  ROS_INFO("%s bbox_msg stamp value is: %f", vehicle.c_str(), sync_bbox_msgs.header.stamp.toSec());
  */

  sync_yolo_pub.publish(sync_img_yolo);
  sync_depth_pub.publish(sync_img_depth);
  sync_bbox_pub.publish(sync_bbox_msgs);
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
      u = (float)(bbox_data[i+2] + bbox_data[i+4])/2;
      v = (float)(bbox_data[i+1] + bbox_data[i+3])/2;
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
  
  depth += cv_ptr->image.at<float>(u, v);
  depth += cv_ptr->image.at<float>(u+1, v);
  depth += cv_ptr->image.at<float>(u-1, v);
  depth += cv_ptr->image.at<float>(u, v+1);
  depth += cv_ptr->image.at<float>(u, v-1);
  depth /=5;

  return depth;
}

void Image_process::set_topic(string group_ns)
{
  std::stringstream ss_yolo, ss_depth, ss_bbox, ss_points;

  ss_yolo << "/" << group_ns << "/yolov7/yolov7/visualization";
  ss_depth << "/" << group_ns << "/camera/depth/image_raw";
  ss_bbox << "/" << group_ns << "/yolov7/yolov7/boundingBox";  

  yolo_input_topic = ss_yolo.str();
  depth_input_topic = ss_depth.str();
  bbox_input_topic = ss_bbox.str();

  cout << "[" << group_ns << " Message_synchronizer]: Input topic was set:\n"
                          << yolo_input_topic << endl 
                          << depth_input_topic << endl
                          << bbox_input_topic << endl;

  ss_yolo.str("");
  ss_depth.str("");
  ss_bbox.str("");
  ss_yolo << "/" << group_ns << "/synchronizer/yolov7/visualization";
  ss_depth << "/" << group_ns << "/synchronizer/camera/depth/image_raw";
  ss_bbox << "/" << group_ns << "/synchronizer/yolov7/boundingBox";
  yolo_output_topic = ss_yolo.str();
  depth_output_topic = ss_depth.str();
  bbox_output_topic = ss_bbox.str();

  cout << "[" << group_ns << " Message_synchronizer]: Output topic was set:\n"
                          << yolo_output_topic << endl 
                          << depth_output_topic << endl
                          << bbox_output_topic << "\n\n";
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
