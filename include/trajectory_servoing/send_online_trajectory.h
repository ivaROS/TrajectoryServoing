#ifndef SEND_ONLINE_TRAJECTORY_H
#define SEND_ONLINE_TRAJECTORY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_path_msg/tf2_path_msg.h>

#include <pips_trajectory_msgs/trajectory_points.h>
#include <turtlebot_trajectory_functions/path.h>
#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <trajectory_servoing/cubic_spline_interpolator.h>

#include <trajectory_servoing/OnlineTrajService.h>

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
typedef std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;

namespace trajectory_servoing_benchmark
{
	class OnlineTrajSender
	{
	public:
        typedef turtlebot_trajectory_generator::ni_state state_type;
        typedef turtlebot_trajectory_generator::ni_controller traj_func_type;
        typedef trajectory_generator::trajectory_states<state_type, traj_func_type> traj_type;
        typedef std::shared_ptr<traj_type> traj_type_ptr;

	public:
		OnlineTrajSender(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf::TransformListener& tf);
		~OnlineTrajSender();

		bool onInit();

		bool sendNiTraj(trajectory_servoing::OnlineTrajService::Request& req, 
						trajectory_servoing::OnlineTrajService::Response& res);

	private:
		bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
		// bool isQuaternionValid(const geometry_msgs::Quaternion& q);
        geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
        nav_msgs::Path generateLocalTraj(const nav_msgs::Path& global_path);
        pips_trajectory_msgs::trajectory_points generateNITraj(const nav_msgs::Path& input_path);

	public:
		tf_buffer_ptr tfBuffer_;
        transform_listener_ptr tf_listener_;

        std::string name_= "online_traj_sender";
        ros::NodeHandle nh_, pnh_;
        ros::ServiceServer service_;
        std::string base_frame_id_, fixed_frame_id_, odom_frame_id_;
        tf::TransformListener& tf_;

        costmap_2d::Costmap2DROS* planner_costmap_ros_;
        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        geometry_msgs::PoseStamped planner_goal_;
        std::string robot_base_frame_, global_frame_;
        bool shutdown_costmaps_;
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

        //set up plan triple buffer
        // std::vector<geometry_msgs::PoseStamped>* planner_plan_;
        // std::vector<geometry_msgs::PoseStamped>* latest_plan_;
        // std::vector<geometry_msgs::PoseStamped>* controller_plan_;
        boost::recursive_mutex traj_mutex_;

        boost::shared_ptr<path_smoothing::CubicSplineInterpolator> traj_csi_;
        trajectory_generator::TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge_;

        ros::Publisher ni_pub_, current_goal_pub_, trajectory_publisher_;

        double v_des_;
	};
}

#endif
