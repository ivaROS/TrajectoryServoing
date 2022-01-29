
#include <ros/ros.h>
#include <ros/topic.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class GTOdomGenerator
{
private:
  ros::NodeHandle nh_, pnh_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
//   tf2_ros::TransformBroadcaster tf_pub_;
  ros::Subscriber gt_sub_;
  ros::Publisher gt_odom_pub_;
  
public:

  GTOdomGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
  {

  }


  void gtCb(const nav_msgs::Odometry::ConstPtr& gt)
  {
    geometry_msgs::TransformStamped transformStamped;
    ros::Time desired_time = gt->header.stamp;
    // ros::Time desired_time = ros::Time::now();
    try{
      transformStamped = tf_buffer_.lookupTransform("odom", desired_time, gt->header.frame_id, gt->header.stamp, "map");
      // transformStamped = tf_buffer_.lookupTransform(gt->header.frame_id, gt->header.stamp, "odom", desired_time, "map");

      geometry_msgs::PoseStamped gt_pose;
      gt_pose.header = gt->header;
      gt_pose.pose = gt->pose.pose;

      geometry_msgs::PoseStamped transformed_gt;
      tf2::doTransform(gt_pose, transformed_gt, transformStamped);

      geometry_msgs::Point curr_vel;
      curr_vel.x = gt->twist.twist.linear.x;
      curr_vel.y = gt->twist.twist.linear.y;
      curr_vel.z = gt->twist.twist.linear.z;
      geometry_msgs::Point transformed_vel;
      geometry_msgs::TransformStamped rot_trans = transformStamped;
      rot_trans.transform.translation.x = 0;
      rot_trans.transform.translation.y = 0;
      rot_trans.transform.translation.z = 0;
      tf2::doTransform(curr_vel, transformed_vel, rot_trans);

      nav_msgs::Odometry gt_odom;
      gt_odom.header = transformed_gt.header;
      gt_odom.child_frame_id = gt->child_frame_id;
      gt_odom.pose.pose = transformed_gt.pose;
      gt_odom.pose.pose.position.z = 0;
      gt_odom.pose.covariance = gt->pose.covariance;
      gt_odom.twist.twist.linear.x = transformed_vel.x;
      // gt_odom.twist.twist.linear.y = transformed_vel.y;
      gt_odom.twist.twist.linear.y = 0;
      gt_odom.twist.twist.linear.z = 0;
      gt_odom.twist.twist.angular = gt->twist.twist.angular;
      gt_odom.twist.twist.angular.x = 0;
      gt_odom.twist.twist.angular.y = 0;
      gt_odom.twist.covariance = gt->twist.covariance;

      gt_odom_pub_.publish(gt_odom);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    
  }
  
  void init()
  {
    gt_sub_ = nh_.subscribe("/ground_truth/state", 10, &GTOdomGenerator::gtCb, this);

    gt_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gt_odom", 1);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"gt_odom_generator");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  GTOdomGenerator pub(nh,pnh);
  pub.init();
  
  ros::spin();
}
