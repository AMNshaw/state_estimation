#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include "ros/param.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace message_filters;

class Time_sync
{
private:
  ros::Publisher sync_yolo_pub;
  ros::Publisher sync_depth_pub;

  sensor_msgs::Image sync_img_yolo;
  sensor_msgs::Image sync_img_depth;

  string yolo_input_topic;
  string depth_input_topic;
  string yolo_output_topic;
  string depth_output_topic;
  string vehicle;

  bool start;

  void sync_cb(const sensor_msgs::ImageConstPtr& ori_yolo, const sensor_msgs::ImageConstPtr& ori_depth);
public:
  Time_sync(ros::NodeHandle &nh, string group_ns);
  ~Time_sync();

  void set_topic(string group_ns);
};

Time_sync::Time_sync(ros::NodeHandle &nh, string group_ns)
{
  start = false;
  vehicle = group_ns;
  set_topic(group_ns);

  sync_yolo_pub = nh.advertise<sensor_msgs::Image>(yolo_output_topic, 1);
  sync_depth_pub = nh.advertise<sensor_msgs::Image>(depth_output_topic, 1);

  message_filters::Subscriber<sensor_msgs::Image> img_yolo_sub(nh, yolo_input_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> img_depth_sub(nh, depth_input_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), img_yolo_sub, img_depth_sub);
  sync.registerCallback(boost::bind(&Time_sync::sync_cb, this, _1, _2));

  ros::spin();
}

Time_sync::~Time_sync(){}

void Time_sync::sync_cb(const sensor_msgs::ImageConstPtr& ori_yolo, const sensor_msgs::ImageConstPtr& ori_depth)
{
  if(!start)
  {
    start = true;
    cout << "[" << vehicle << " Message_synchronizer]: Start synchronizing messages... \n\n";
  }

  sync_img_yolo = *ori_yolo;
  sync_img_depth = *ori_depth;

  //cout << "*******************" << endl;
  //ROS_INFO("%s image_yolo stamp value is: %f", vehicle.c_str(), sync_img_yolo.header.stamp.toSec());
  //ROS_INFO("%s image_depth stamp value is: %f", vehicle.c_str(), sync_img_depth.header.stamp.toSec());

  sync_yolo_pub.publish(sync_img_yolo);
  sync_depth_pub.publish(sync_img_depth);
}

void Time_sync::set_topic(string group_ns)
{
  std::stringstream ss_yolo, ss_depth;

  ss_yolo << "/" << group_ns << "/yolov7/yolov7/visualization";
  ss_depth << "/" << group_ns << "/camera/depth/image_raw";
  yolo_input_topic = ss_yolo.str();
  depth_input_topic = ss_depth.str();

  cout << "[" << group_ns << " Message_synchronizer]: Input topic was set:\n" << yolo_input_topic << endl << depth_input_topic << endl;

  ss_yolo.str("");
  ss_depth.str("");
  ss_yolo << "/" << group_ns << "/synchronized/yolov7/visualization";
  ss_depth << "/" << group_ns << "/synchronized/camera/depth/image_raw";
  yolo_output_topic = ss_yolo.str();
  depth_output_topic = ss_depth.str();

  cout << "[" << group_ns << " Message_synchronizer]: Output topic was set:\n" << yolo_output_topic << endl << depth_output_topic << "\n\n";
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_synchronizer");
  ros::NodeHandle nh;

  string group_ns;
  ros::param::get("vehicle", group_ns);

  Time_sync sync(nh, group_ns);
  
  

  return 0;
}
