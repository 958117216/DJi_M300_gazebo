#include "DJi_M300_gazebo/DJi_M300_gazebo_bridge.h"



std::string model_name = "DJi_M300";
std::string reference_frame = "world";


void LAVcallback(const geometry_msgs::PointStampedConstPtr& position_msg, const geometry_msgs::QuaternionStampedConstPtr& attitude_quaternion_msg,const geometry_msgs::Vector3StampedConstPtr& velocity_msg)
{
 
  target_pose.position.x = position_msg->point.x;
  target_pose.position.y = position_msg->point.y;
  target_pose.position.z = position_msg->point.z;

  target_pose.orientation.w = attitude_quaternion_msg->quaternion.w;
  target_pose.orientation.x = attitude_quaternion_msg->quaternion.x;
  target_pose.orientation.y = attitude_quaternion_msg->quaternion.y;
  target_pose.orientation.z = attitude_quaternion_msg->quaternion.z;

  target_twist.linear.x  = velocity_msg->vector.x;
  target_twist.linear.y  = velocity_msg->vector.y;
  target_twist.linear.z  = velocity_msg->vector.z;
 
  //target_twist.angular.x = 0;
  //target_twist.angular.y = 0;
  //target_twist.angular.z = 0;

   target_model_state.model_name = model_name;
   target_model_state.reference_frame = reference_frame;
   target_model_state.pose = target_pose;
   target_model_state.twist = target_twist;
   set_model_state.request.model_state = target_model_state;
   model_state_client.call(set_model_state);   //call gazebo service

   odom.header.stamp=ros::Time::now();//position_msg->header.stamp;
   odom.header.frame_id="world";
   odom.pose.pose=target_pose;
   odom.twist.twist=target_twist;
   odom_pub_.publish(odom);
   
   // 创建tf的广播器
   static tf::TransformBroadcaster br;
 
   //初始化tf数据
   tf::Transform transform;
   transform.setOrigin(tf::Vector3(position_msg->point.x,position_msg->point.y,position_msg->point.z));
   tf::Quaternion q;
   q[3]=attitude_quaternion_msg->quaternion.w;
   q[0]=attitude_quaternion_msg->quaternion.x;
   q[1]=attitude_quaternion_msg->quaternion.y;
   q[2]=attitude_quaternion_msg->quaternion.z;
   transform.setRotation(q);

   //广播world与无人机坐标系之间的tf数据
   br.sendTransform(tf::StampedTransform(transform,position_msg->header.stamp, "world", "base_link")); //position_msg->header.stamp
   br.sendTransform(tf::StampedTransform(transform,position_msg->header.stamp, "world", "3Dlidar_16_link"));
}

//会在这个节点汇总从大疆得到的位置、位姿、速度、角速度，并进行同步整合发布形式为nav_msgs的odom话题
//发布base_link到world坐标下的tf变换（最终导航时应去掉该tf，应转交给vio发布相关变换）
int main(int argc, char **argv)
{

  ros::init(argc, argv, "DJi_M300_gazebo_bridge");

  ros::NodeHandle nh;

//初始化订阅
  localPosition_sub_.reset(new message_filters::Subscriber<geometry_msgs::PointStamped>(nh,"dji_osdk_ros/local_position",100));
  attitude_sub_.reset(new message_filters::Subscriber<geometry_msgs::QuaternionStamped>(nh,"dji_osdk_ros/attitude",100));
  velocity_sub_.reset(new message_filters::Subscriber<geometry_msgs::Vector3Stamped>(nh,"dji_osdk_ros/velocity",100));
 
  sync_localpos_atti_v_.reset(new message_filters::Synchronizer<SyncPolicyLAV>
   (SyncPolicyLAV(100),*localPosition_sub_,*attitude_sub_,*velocity_sub_));
  sync_localpos_atti_v_->registerCallback(boost::bind(&LAVcallback, _1, _2,_3));

//初始化发布
   odom_pub_ = nh.advertise<nav_msgs::Odometry>("/DJI_M300_bridge/odom", 10);

//初始化gazebo服务
  ros::service::waitForService("/gazebo/set_model_state");
  model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);
  ROS_INFO("Bridge between PC sim and gazebo connected");


  ros::spin();//循环进回调


  return 0;
}
