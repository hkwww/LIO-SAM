#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <gps_convert_utils.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Eigen>

Eigen::Vector3d first_pose;
bool first_pose_flag = false;

ros::Publisher pub_odom;

void GPSHandler(const sensor_msgs::NavSatFix::ConstPtr& rtk_msg) {
  double latitude = rtk_msg->latitude;
  double longitude = rtk_msg->longitude;
  double altitude = rtk_msg->altitude;

  // 转化为utm坐标系
  UTMCoor utmcoor;
  LatLonToUTMXY(latitude / 180.0 * M_PI, longitude / 180.0 * M_PI, 51, utmcoor);
  Eigen::Vector3d gps_utm(utmcoor.x, utmcoor.y, altitude);
  if (!first_pose_flag) {
    first_pose = gps_utm;
    first_pose_flag = true;
  }

  gps_utm -= first_pose;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/map";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = gps_utm(0);
  odom_msg.pose.pose.position.y = gps_utm(1);
  odom_msg.pose.pose.position.z = gps_utm(2);
  odom_msg.pose.covariance[0] = rtk_msg->position_covariance[0];
  odom_msg.pose.covariance[7] = rtk_msg->position_covariance[4];
  odom_msg.pose.covariance[14] = rtk_msg->position_covariance[8];

  pub_odom.publish(odom_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_visualization");
  ros::NodeHandle n;

  ros::Publisher pub_gps_path = n.advertise<nav_msgs::Path>("/gps_path", 10);
  pub_odom = n.advertise<nav_msgs::Odometry>("/gps_odom", 10);

  std::string bag_path = "/home/hkw/rosbag/rs_ruby_mti_680_new.bag";
  std::string gps_topic = "/gnss";

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(gps_topic);

  rosbag::View view_;
  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  ros::Time time_finish = view_full.getEndTime();

  view_.addQuery(bag, rosbag::TopicQuery(topics), time_init, time_finish);
  if (view_.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return 0;
  }

  ros::Rate rate(10);
  for (const rosbag::MessageInstance& m : view_) {
    ros::Time ros_bag_time = m.getTime();
    if (m.getTopic() == gps_topic) {
      sensor_msgs::NavSatFix::ConstPtr gps_msg =
          m.instantiate<sensor_msgs::NavSatFix>();
      if (gps_msg != NULL) {
        GPSHandler(gps_msg);
        ros::spinOnce();
        rate.sleep();
      }
    }
  }

  return 0;
}
