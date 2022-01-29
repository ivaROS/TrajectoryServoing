#ifndef TRAJECTORY_SERVO_NAVIGATOR_H
#define TRAJECTORY_SERVO_NAVIGATOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <sparse_stereo_msgs/TrackedPoint.h>
#include <sparse_stereo_msgs/TrackedPointList.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_path_msg/tf2_path_msg.h>
#include <tf2_pips_traj_msg/tf2_pips_traj_msg.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <pips_trajectory_msgs/trajectory_points.h>
#include <turtlebot_trajectory_functions/path.h>
#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include "trajectory_servoing/trajectory_servo_core.h"
#include "trajectory_servoing/trajectory_servo_feedforward.h"
#include "trajectory_servoing/cubic_spline_interpolator.h"

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
typedef std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

namespace trajectory_servo_navigator
{
    class StereoVSNavigator
    {
    public:
        typedef turtlebot_trajectory_generator::ni_state state_type;
        typedef turtlebot_trajectory_generator::ni_controller traj_func_type;
        typedef trajectory_generator::trajectory_states<state_type, traj_func_type> traj_type;
        typedef std::shared_ptr<traj_type> traj_type_ptr;
    public:
        StereoVSNavigator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        ~StereoVSNavigator(){};
        virtual bool onInit();
        pips_trajectory_msgs::trajectory_points generateNITraj(const nav_msgs::Path& input_path, const std_msgs::Header& features_header);
        
        cv::Mat drawFeaturesImg(const sparse_stereo_msgs::TrackedPointList& orig, const sparse_stereo_msgs::TrackedPointList& filtered, bool enable_matched_color = true);
        
        tf_buffer_ptr tfBuffer_;
        transform_listener_ptr tf_listener_;
        
        boost::shared_ptr<image_geometry::StereoCameraModel> stereo_camera_model_;
        boost::shared_ptr<trajectory_servo_core::VisualServoController> vs_controller_;
        boost::shared_ptr<path_smoothing::CubicSplineInterpolator> traj_csi_;
        
        nav_msgs::Path getLocalTrajLocalCopy()
        {
            boost::mutex::scoped_lock lock(traj_mutex_);
            nav_msgs::Path local_copy = curr_traj_;
            return local_copy;
        }
        
        pips_trajectory_msgs::trajectory_points getLocalNITrajLocalCopy()
        {
            boost::mutex::scoped_lock lock(traj_mutex_);
            pips_trajectory_msgs::trajectory_points local_copy = curr_ni_traj_;
            return local_copy;
        }
        
    public:
        void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
//         void trajCB(const nav_msgs::Path::ConstPtr& traj_msg);
        void trajCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& traj_msg);
        void featuresCB(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const sensor_msgs::CameraInfo::ConstPtr& l_info,
                                       const sensor_msgs::CameraInfo::ConstPtr& r_info);
        
        
    protected:
        std::string name_= "stereo_visual_servo_navigator";
        ros::NodeHandle nh_, pnh_;
        std::string base_frame_id_, fixed_frame_id_, odom_frame_id_, features_frame_id_;
        
        boost::mutex traj_mutex_;
        ros::Subscriber traj_sub_, ni_traj_sub_, odom_sub_;
        
        message_filters::Subscriber<sparse_stereo_msgs::TrackedPointList> features_sub_;
        message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
//         typedef ExactTime<sparse_stereo_msgs::TrackedPointList, CameraInfo> ExactPolicy;
        typedef ApproximateTime<sparse_stereo_msgs::TrackedPointList, CameraInfo, CameraInfo> ApproximatePolicy;
//         typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
//         boost::shared_ptr<ExactSync> exact_sync_;
        boost::shared_ptr<ApproximateSync> approximate_sync_;
        
        ros::Publisher odom_reset_pub_, curr_local_traj_pub_, command_publisher_, command_timed_pub_;

        // Number of regeneration
        int num_regeneration_;
        ros::Publisher num_regeneration_pub_;
        
        // Debug
        ros::Publisher filtered_curr_features_img_, filtered_desired_features_img_;
        ros::Publisher next_goal_pub_;
        
        nav_msgs::Path curr_traj_;
        pips_trajectory_msgs::trajectory_points curr_ni_traj_;
        nav_msgs::Odometry::ConstPtr curr_odom_;
        geometry_msgs::TransformStamped curr_camera_base_trans_;
        
        // near identity traj generation
        float v_des_;
        trajectory_generator::TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge_;
        ros::Publisher trajectory_publisher_;
        
        bool traj_updated_;
        
        // Flags for different approaches
        bool continuous_type_, ni_traj_enabled_;

        geometry_msgs::Twist prev_cmd_;
        
    };
    
}

#endif // VISUAL_SERVO_NAVIGATOR_H
