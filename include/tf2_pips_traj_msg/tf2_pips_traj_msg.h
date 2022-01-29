// This file could replace the one in pips_trajectory_testing, but I don't think that the tf dependencies should be required just to use the messages

#ifndef TF2_PIPS_TRAJ_MSG_H
#define TF2_PIPS_TRAJ_MSG_H

// #include <nav_msgs/Path.h>
#include <pips_trajectory_msgs/trajectory_points.h>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Eigen>
#include <tf2/convert.h>  //needed to give access to templates
//#include <tf2_eigen/tf2_eigen.h>  //needed to transform to eigen
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace tf2
{

  
// method to extract timestamp from object
template <>
inline
  const ros::Time& getTimestamp(const pips_trajectory_msgs::trajectory_points& t) {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
  const std::string& getFrameId(const pips_trajectory_msgs::trajectory_points& t) {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const pips_trajectory_msgs::trajectory_points& t_in, pips_trajectory_msgs::trajectory_points& t_out, const geometry_msgs::TransformStamped& transform_stamped)
  {
    //NOTE: It might be more efficient to perform the conversion only once, but not going to worry about it at this point
    //tf2::Transform t;
    //fromMsg(transform_stamped.transform, t);
    
    for(size_t i = 0; i < t_in.points.size(); i++)
    {
        pips_trajectory_msgs::trajectory_point point_in = t_in.points[i];
        geometry_msgs::PoseStamped pose_in;
        pose_in.pose.position.x = point_in.x;
        pose_in.pose.position.y = point_in.y;
        pose_in.pose.position.z = 0;
        tf2::Quaternion in_q;
        in_q.setRPY( 0, 0, point_in.theta );
        geometry_msgs::Quaternion quat_msg = tf2::toMsg(in_q);
        pose_in.pose.orientation = quat_msg;
        
        geometry_msgs::PoseStamped pose_out;
        
        doTransform(pose_in, pose_out, transform_stamped);
        
        pips_trajectory_msgs::trajectory_point point_out;
        point_out.time = point_in.time;
        point_out.v = point_in.v;
        point_out.w = point_in.w;
        point_out.x = pose_out.pose.position.x;
        point_out.y = pose_out.pose.position.y;
        tf2::Quaternion out_q;
        tf2::convert(pose_out.pose.orientation , out_q);
        tf2::Matrix3x3 m(out_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        point_out.theta = yaw;
        t_out.points.push_back(point_out);
    }
    
    
    t_out.header.stamp = transform_stamped.header.stamp; 
    t_out.header.frame_id = transform_stamped.header.frame_id;
  }


}

#endif /* TF2_PIPS_TRAJ_MSG_H */

