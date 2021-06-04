#ifndef DJI_M300_GAZEBO_BRIDGE_H
#define DJI_M300_GAZEBO_BRIDGE_H

#include "ros/ros.h"
#include <ros/console.h>

#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf/transform_broadcaster.h>

#include <cmath>
#include <string>

#include <nav_msgs/Odometry.h>

using namespace std;

//订阅话题
  shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped>> localPosition_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::QuaternionStamped>> attitude_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::Vector3Stamped>> velocity_sub_;
//shared_ptr<message_filters::Subscriber<geometry_msgs::Vector3Stamped>> angularvelocity_sub_;/ /暂没发现相关消息

//定义同步策略
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::QuaternionStamped,geometry_msgs::Vector3Stamped>  SyncPolicyLAV;

//定义同步器
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyLAV>> SynchronizerLAV;
  SynchronizerLAV  sync_localpos_atti_v_;

//定义发布话题及其内容
  ros::Publisher odom_pub_;

  nav_msgs::Odometry odom;

//定义gazebo服务
  ros::ServiceClient model_state_client;

  gazebo_msgs::SetModelState set_model_state;
  gazebo_msgs::ModelState target_model_state;
  geometry_msgs::Pose target_pose;
  geometry_msgs::Twist target_twist;


#endif
