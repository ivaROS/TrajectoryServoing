#include <trajectory_servoing/trajectory_servo_navigator.h>

namespace trajectory_servo_navigator
{
    StereoVSNavigator::StereoVSNavigator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh)
    {
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
        
        stereo_camera_model_ = boost::make_shared<image_geometry::StereoCameraModel>();
        
        traj_updated_ = false;
    }
    
    bool StereoVSNavigator::onInit()
    {
        std::string traj_topic = "/plan";
        std::string ni_traj_topic = "/ni_traj";
        std::string features_topic = "/features";
        std::string odom_topic = "/odom";
        std::string curr_local_traj_pub_topic = "/vs/local_traj";
        base_frame_id_ = "base_footprint";
        fixed_frame_id_ = "map";
        odom_frame_id_ = "odom";
        features_frame_id_ = "stereo_camera_optical_frame";
        
        pnh_.getParam("traj_topic", traj_topic);
        pnh_.getParam("ni_traj_topic", ni_traj_topic);
        pnh_.getParam("features_topic", features_topic);
        pnh_.getParam("odom_topic", odom_topic);
        pnh_.getParam("local_traj_topic", curr_local_traj_pub_topic);
        pnh_.getParam("base_frame_id", base_frame_id_);
        pnh_.getParam("fixed_frame_id", fixed_frame_id_);
        pnh_.getParam("odom_frame_id", odom_frame_id_);
        pnh_.getParam("features_frame_id", features_frame_id_);
        
        pnh_.setParam("traj_topic", traj_topic);
        pnh_.setParam("ni_traj_topic", ni_traj_topic);
        pnh_.setParam("features_topic", features_topic);
        pnh_.setParam("odom_topic", odom_topic);
        pnh_.setParam("local_traj_topic", curr_local_traj_pub_topic);
        pnh_.setParam("base_frame_id", base_frame_id_);
        pnh_.setParam("fixed_frame_id", fixed_frame_id_);
        pnh_.setParam("odom_frame_id", odom_frame_id_);
        pnh_.setParam("features_frame_id", features_frame_id_);
        
        float const_linear = 0.05;
        float lambda = 3;
        double milestone_thresh = 0.2;
        double final_goal_thresh = 0.3;
        float tx = 0.1;
        
        pnh_.getParam("const_linear", const_linear);
        pnh_.getParam("lambda", lambda);
        pnh_.getParam("milestone_thresh", milestone_thresh);
        pnh_.getParam("final_goal_thresh", final_goal_thresh);
        pnh_.getParam("tx", tx);
        
        pnh_.setParam("const_linear", const_linear);
        pnh_.setParam("lambda", lambda);
        pnh_.setParam("milestone_thresh", milestone_thresh);
        pnh_.setParam("final_goal_thresh", final_goal_thresh);
        pnh_.setParam("tx", tx);
        
        int pointsPerUnit = 5;
        int skipPoints = 0;
        bool useEndConditions = false;
        bool useMiddleConditions = false;
        
        pnh_.getParam("pointsPerUnit", pointsPerUnit);
        pnh_.getParam("skipPoints", skipPoints);
        pnh_.getParam("useEndConditions", useEndConditions);
        pnh_.getParam("useMiddleConditions", useMiddleConditions);
        
        pnh_.setParam("pointsPerUnit", pointsPerUnit);
        pnh_.setParam("skipPoints", skipPoints);
        pnh_.setParam("useEndConditions", useEndConditions);
        pnh_.setParam("useMiddleConditions", useMiddleConditions);
        
        continuous_type_ = false;
        ni_traj_enabled_ = false;
        
        pnh_.getParam("continuous_type", continuous_type_);
        pnh_.getParam("ni_traj_enabled", ni_traj_enabled_);
        
        pnh_.setParam("continuous_type", continuous_type_);
        pnh_.setParam("ni_traj_enabled", ni_traj_enabled_);
        
        
        odom_sub_ = nh_.subscribe(odom_topic, 1, &StereoVSNavigator::odomCB, this);
//         traj_sub_ = nh_.subscribe(traj_topic, 10, &StereoVSNavigator::trajCB, this);
        ni_traj_sub_ = nh_.subscribe(ni_traj_topic, 10, &StereoVSNavigator::trajCB, this);
        
        features_sub_.subscribe(nh_, features_topic, 10);
        sub_l_info_.subscribe(nh_, "/multisense_sl/camera/left/camera_info", 1);
        sub_r_info_.subscribe(nh_, "/multisense_sl/camera/right/camera_info", 1);
        
        int queue_size;
        pnh_.param("queue_size", queue_size, 12);
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                    features_sub_, sub_l_info_, sub_r_info_) );
        approximate_sync_->registerCallback(boost::bind(&StereoVSNavigator::featuresCB,
                                                        this, _1, _2, _3));
        
        odom_reset_pub_ = nh_.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);
        curr_local_traj_pub_ = nh_.advertise<nav_msgs::Path>(curr_local_traj_pub_topic, 1);
        
//         trajectory_publisher_ = nh_.advertise< nav_msgs::Path >("/vs/desired_trajectory", 10);
        
        command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 1);
        command_timed_pub_ = nh_.advertise< geometry_msgs::TwistStamped >("cmd_vel_mux/input/navi/timed", 1);
        
        // Debug
        filtered_curr_features_img_ = nh_.advertise<sensor_msgs::Image>("/vs/filtered_curr_features_img", 1); 
        filtered_desired_features_img_ = nh_.advertise<sensor_msgs::Image>("/vs/filtered_desired_features_img", 1);
        next_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vs/next_goal", 1);

        num_regeneration_pub_ = nh_.advertise<std_msgs::UInt32>("/vs/num_regeneration", 1);
        
        // Flags, modes and params
        int goal_check_mode = 0;
        bool uncertainty_enabled = false;
        double sigmaMu = 1;
        double sigmaNu = 1;
        double sigmaD = 0.15;
        double noise_std=0;
        
        bool feat_traj_reg_enabled = true;
        int feat_traj_reg_mode = 1;
        int feat_traj_reg_thresh = 20;
        
        pnh_.getParam("goal_check_mode", goal_check_mode);
        pnh_.getParam("uncertainty_enabled", uncertainty_enabled);
        pnh_.getParam("sigmaMu", sigmaMu);
        pnh_.getParam("sigmaNu", sigmaNu);
        pnh_.getParam("sigmaD", sigmaD);
        pnh_.getParam("feat_traj_reg_enabled", feat_traj_reg_enabled);
        pnh_.getParam("feat_traj_reg_mode", feat_traj_reg_mode);
        pnh_.getParam("feat_traj_reg_thresh", feat_traj_reg_thresh);
        pnh_.getParam("noise_std", noise_std);
        
        pnh_.setParam("goal_check_mode", goal_check_mode);
        pnh_.setParam("uncertainty_enabled", uncertainty_enabled);
        pnh_.setParam("sigmaMu", sigmaMu);
        pnh_.setParam("sigmaNu", sigmaNu);
        pnh_.setParam("sigmaD", sigmaD);
        pnh_.setParam("feat_traj_reg_enabled", feat_traj_reg_enabled);
        pnh_.setParam("feat_traj_reg_mode", feat_traj_reg_mode);
        pnh_.setParam("feat_traj_reg_thresh", feat_traj_reg_thresh);
        pnh_.setParam("noise_std", noise_std);
        
        trajectory_servo_core::params flags_and_modes(goal_check_mode,
                                                      uncertainty_enabled,
                                                      feat_traj_reg_enabled, feat_traj_reg_mode, feat_traj_reg_thresh);

        bool feedforward_enabled = false;
        pnh_.getParam("feedforward_enabled", feedforward_enabled);
        pnh_.setParam("feedforward_enabled", feedforward_enabled);

        bool speedcontrol_enabled = false;
        double speedcontrol_p0 = 900000;
        pnh_.getParam("speedcontrol_enabled", speedcontrol_enabled);
        pnh_.getParam("speedcontrol_p0", speedcontrol_p0);
        pnh_.setParam("speedcontrol_enabled", speedcontrol_enabled);
        pnh_.setParam("speedcontrol_p0", speedcontrol_p0);

        if(!feedforward_enabled)
        {
            vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoControllerFeedforward>(tfBuffer_);
            
            if(!vs_controller_->initialized_)
            {
                vs_controller_->init();
            }
        }
        else if(feedforward_enabled)
        {
            vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoController>(tfBuffer_);
            
            if(!vs_controller_->initialized_)
            {
                vs_controller_->init();
            }
        }
        else
        {
            ROS_ERROR_STREAM("No valid controller for these configurations");
        }
        
        
        if(vs_controller_->initialized_)
        {
            vs_controller_->setConstLinearVel(const_linear);
            vs_controller_->setLambda(lambda);
            vs_controller_->setMilestoneThresh(milestone_thresh);
            vs_controller_->setFinalGoalThresh(final_goal_thresh);
            vs_controller_->setTx(tx);
            vs_controller_->setFlagsAndModes(flags_and_modes);
            vs_controller_->setNoiseStd(noise_std);
        }
            
        v_des_ = vs_controller_->getConstLinearVel();
        traj_csi_ = boost::make_shared<path_smoothing::CubicSplineInterpolator>(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
        num_regeneration_ = 0;
        
        return true;
    }
    
    void StereoVSNavigator::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        curr_odom_ = msg;
    }
    
//     void StereoVSNavigator::trajCB(const nav_msgs::Path::ConstPtr& traj_msg)
//     {
//         boost::mutex::scoped_lock lock(traj_mutex_);
//         
//         // Reset turtlebot odom frame
//         std_msgs::Empty empty;
//         odom_reset_pub_.publish(empty);
//         ros::Duration(0.5).sleep();
//         
//         // Transform the plan to local frame and use cubic spline to interpolate.
//         nav_msgs::Path local_traj;
//         local_traj = tfBuffer_->transform(*traj_msg, odom_frame_id_, ros::Time(0), fixed_frame_id_);
//         traj_csi_->interpolatePath(local_traj, curr_traj_);
//         ROS_INFO_STREAM("Num of poses on the local traj: " << curr_traj_.poses.size() << " in frame: " << curr_traj_.header.frame_id << " and " << curr_traj_.poses[0].header.frame_id);
//         traj_updated_ = true;
//         
//         curr_camera_base_trans_ = tfBuffer_->lookupTransform(base_frame_id_, features_frame_id_, ros::Time(0), ros::Duration(1));
//         vs_controller_->registerCameraBaseTrans(curr_camera_base_trans_);
//         
//         // Publish the local traj for debugging
//         if(!curr_traj_.header.frame_id.empty())
//            curr_local_traj_pub_.publish(curr_traj_);
//     }
    
    void StereoVSNavigator::trajCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& traj_msg)
    {
        boost::mutex::scoped_lock lock(traj_mutex_);
        
        if(traj_msg->points.size() < 2)
            return;
        
        ROS_INFO_STREAM("RECEIVED TRAJECTORY.");
        
        // Transform the ni traj to odom frame.
        pips_trajectory_msgs::trajectory_points local_traj;
        local_traj = tfBuffer_->transform(*traj_msg, odom_frame_id_, ros::Time(0), fixed_frame_id_);
//         traj_csi_->interpolatePath(local_traj, curr_traj_);
        curr_ni_traj_ = local_traj;
        ROS_INFO_STREAM("Num of poses on the local traj: " << curr_ni_traj_.points.size() << " in frame: " << curr_ni_traj_.header.frame_id);
        traj_updated_ = true;
        
        curr_camera_base_trans_ = tfBuffer_->lookupTransform(base_frame_id_, features_frame_id_, ros::Time(0), ros::Duration(1));
        vs_controller_->registerCameraBaseTrans(curr_camera_base_trans_);
        
        // Publish the local traj for debugging
        nav_msgs::Path curr_traj;
        curr_traj.header = curr_ni_traj_.header;
        for(size_t i = 0; i < curr_ni_traj_.points.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = curr_ni_traj_.points[i].x;
            pose.pose.position.y = curr_ni_traj_.points[i].y;
            pose.pose.position.z = 0;
            
            pose.pose.orientation = traj_gen_bridge_.yawToQuaternion(curr_ni_traj_.points[i].theta);
            
            curr_traj.poses.push_back(pose);
        }
        
        if(!curr_traj.header.frame_id.empty())
           curr_local_traj_pub_.publish(curr_traj);
    }
    
    void StereoVSNavigator::featuresCB(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features,
                                       const sensor_msgs::CameraInfo::ConstPtr& l_info,
                                       const sensor_msgs::CameraInfo::ConstPtr& r_info)
    {
        ROS_DEBUG_STREAM_NAMED(name_, "Features received.");
        // Update stereo camera info
        ros::WallTime start = ros::WallTime::now();
        stereo_camera_model_->fromCameraInfo(l_info, r_info);
        vs_controller_->registerStereoCameraModel(stereo_camera_model_);
        ROS_INFO_STREAM("Info registered time: " << (double)(ros::WallTime::now() - start).toNSec() / 1000000 << " ms.");
        
        // Update traj if needed
        if(traj_updated_)
        {
            start = ros::WallTime::now();
            // Make a local copy of the local trajectory
            pips_trajectory_msgs::trajectory_points local_traj = getLocalNITrajLocalCopy();
            
            // Register local traj to the visual servo core
            ROS_INFO_STREAM_NAMED(name_, "New traj set. Initialization.");
            vs_controller_->initialization(local_traj);
            
            if(continuous_type_)
            {
                if(ni_traj_enabled_)
                {
                    ROS_INFO_STREAM("Generate feature trajectory.");
//                     pips_trajectory_msgs::trajectory_points ni_traj = generateNITraj(local_traj, features->header);
//                     ROS_INFO_STREAM(ni_traj.points[0].x << " " << ni_traj.points[0].y << " " << ni_traj.points[0].theta << " " << ni_traj.points[0].time);
                    vs_controller_->generateAllFeatures(features, local_traj, curr_odom_);
                }
                else
                {
                    
                }
                
//                 vs_controller_->updateNextGoal();
            }
            else
            {
                vs_controller_->updateNextGoal();
            }
            
            // Remove the traj updated flag
            traj_updated_ = false;
            ROS_INFO_STREAM("Feature traj generates time: " << (double)(ros::WallTime::now() - start).toNSec() / 1000000 << " ms.");
        }
        
        geometry_msgs::Twist cmd;
        if(!vs_controller_->goalReached())
        {
            if(continuous_type_)
            {
                if(ni_traj_enabled_)
                {
                    double pre_time_fraction;
                    start = ros::WallTime::now();
                    vs_controller_->run_image(features, curr_odom_, pre_time_fraction, cmd);
                    ROS_INFO_STREAM("Servo time: " << (double)(ros::WallTime::now() - start).toNSec() / 1000000 << " ms.");
                    if(vs_controller_->checkRegeneration(vs_controller_->getNumFilteredFeatures()))
                    {
                        start = ros::WallTime::now();
                        // ROS_INFO_STREAM(pre_time_fraction);
                        vs_controller_->featureRegeneration(pre_time_fraction, features, curr_odom_);
                        // cmd = prev_cmd_;
                        cmd.linear.x = vs_controller_->getConstLinearVel();
                        cmd.angular.z = vs_controller_->getDesiredAngularVel();

                        num_regeneration_++;
                        num_regeneration_pub_.publish(num_regeneration_);

                        ROS_INFO_STREAM("Regeneration time: " << (double)(ros::WallTime::now() - start).toNSec() / 1000000 << " ms.");
                    }
                }
                else
                {
                    
                }
            }
            else
            {
                vs_controller_->run(features, curr_odom_, cmd);
            }
            prev_cmd_ = cmd;
            geometry_msgs::TwistStamped timed_cmd;
            timed_cmd.twist = cmd;
            timed_cmd.header.frame_id = "base_footprint";
            timed_cmd.header.stamp = ros::Time::now();
            command_publisher_.publish(cmd);
            command_timed_pub_.publish(timed_cmd);

            ROS_INFO_STREAM_NAMED(name_, "Command: " << cmd.linear.x <<"m/s, " << cmd.angular.z << "rad/s");
        }
        else
        {
            ROS_INFO_STREAM_NAMED(name_, "Goal reached.");
        }
        
        // Debug
        bool vis_enabled = false;
        if(vis_enabled)
        {
            start = ros::WallTime::now();
            geometry_msgs::PoseStamped next_goal;
            vs_controller_->getNextGoal(next_goal);
            next_goal.header = curr_traj_.header;
            next_goal_pub_.publish(next_goal);
            
            if(vs_controller_->local_traj_registered_)
            {
                sparse_stereo_msgs::TrackedPointList curr_features, desired_features, filtered_curr_features, filtered_desired_features;
                vs_controller_->getAllFeatures(curr_features, desired_features, filtered_curr_features, filtered_desired_features);
                
                if(curr_features.tracked_list.size() != 0 && filtered_curr_features.tracked_list.size() != 0)
                {
                    cv::Mat img_curr = drawFeaturesImg(curr_features, filtered_curr_features);
                    cv_bridge::CvImage out_curr_msg;
                    out_curr_msg.header   = features->header; // Same timestamp and tf frame as input image
                    out_curr_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
                    out_curr_msg.image    = img_curr; // Your cv::Mat
                    filtered_curr_features_img_.publish(out_curr_msg.toImageMsg());
                }
                if(desired_features.tracked_list.size() != 0 && filtered_desired_features.tracked_list.size() != 0)
                {
                    bool enable_matched_color = false;
                    cv::Mat img_desired = drawFeaturesImg(desired_features, filtered_desired_features, enable_matched_color);
                    cv_bridge::CvImage out_desired_msg;
                    out_desired_msg.header   = features->header; // Same timestamp and tf frame as input image
                    out_desired_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
                    out_desired_msg.image    = img_desired; // Your cv::Mat
                    filtered_desired_features_img_.publish(out_desired_msg.toImageMsg());
                }
            }
            ROS_INFO_STREAM("Debug and vis time: " << (double)(ros::WallTime::now() - start).toNSec() / 1000000 << " ms.");
        }
        
    }
    
    pips_trajectory_msgs::trajectory_points StereoVSNavigator::generateNITraj(const nav_msgs::Path& input_path, const std_msgs::Header& features_header)
    {
        turtlebot_trajectory_functions::Path::Ptr pathtraj = std::make_shared<turtlebot_trajectory_functions::Path>(input_path, v_des_);
        double tf = pathtraj->getTF();
        trajectory_generator::traj_params_ptr params = std::make_shared<trajectory_generator::traj_params>();
        params->tf=tf;
        
        double v_max=.5;
        double w_max=4;
        double a_max=.55;
        double w_dot_max=1.78;
        
        turtlebot_trajectory_generator::near_identity ni(1,5,1,.01, v_max,w_max,a_max,w_dot_max);    
        traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
        nc->setTrajFunc(pathtraj);
        
        traj_type_ptr traj = std::make_shared<traj_type>();
        traj->header.frame_id = input_path.header.frame_id;
        traj->header.stamp = features_header.stamp;
        traj->trajpntr = nc ;
        traj->params = params;
        
        traj_gen_bridge_.generate_trajectory(traj);
        
        pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toMsg ();
        
        nav_msgs::Path::ConstPtr path_msg = traj->toPathMsg();
        trajectory_publisher_.publish(path_msg);
        
        return trajectory_msg;
    }
    
    cv::Mat StereoVSNavigator::drawFeaturesImg(const sparse_stereo_msgs::TrackedPointList& orig, const sparse_stereo_msgs::TrackedPointList& filtered, bool enable_matched_color)
    {
        cv::Mat img(2*floor(stereo_camera_model_->left().cy()), 2*floor(stereo_camera_model_->left().cx()), CV_8UC3, cv::Scalar(255, 255, 255));
        
        for(size_t i = 0; i < orig.tracked_list.size(); i++)
        {
            sparse_stereo_msgs::TrackedPoint pt_orig = orig.tracked_list[i];
            for(size_t j = 0; j < filtered.tracked_list.size(); j++)
            {
                sparse_stereo_msgs::TrackedPoint pt_filtered = filtered.tracked_list[j];
                if(enable_matched_color && pt_filtered.id == pt_orig.id)
                {
                    if(pt_filtered.u_l >=0 && pt_filtered.u_l <= 2*stereo_camera_model_->left().cx() - 1 && pt_filtered.v_l >=0 && pt_filtered.v_l <= 2*stereo_camera_model_->left().cy() - 1)
                    {
                        cv::circle(img, cv::Point(pt_filtered.u_l, pt_filtered.v_l), 4, cv::Scalar(0, 255, 0), 2);
                    }
                }
                else
                {
                    if(pt_orig.u_l >=0 && pt_orig.u_l <= 2*stereo_camera_model_->left().cx() - 1 && pt_orig.v_l >=0 && pt_orig.v_l <= 2*stereo_camera_model_->left().cy() - 1)
                    {
                        double marker_width = 1;
                        if(enable_matched_color)
                        {
                            marker_width = 1;
                        }
                        cv::circle(img, cv::Point(pt_orig.u_l, pt_orig.v_l), 4, cv::Scalar(255, 0, 0), marker_width);
                    }
                }
            }
        }
        
        return img;
    }
}



