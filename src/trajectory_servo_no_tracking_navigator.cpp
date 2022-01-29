#include <trajectory_servoing/trajectory_servo_no_tracking_navigator.h>

namespace trajectory_servo_navigator
{
    StereoVSNoTrackingNavigator::StereoVSNoTrackingNavigator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        StereoVSNavigator(nh, pnh)
    {
        frame_counter_ = -1;
    }
    
    bool StereoVSNoTrackingNavigator::onInit()
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
        
        
        odom_sub_ = nh_.subscribe(odom_topic, 1, &StereoVSNavigator::odomCB, dynamic_cast<StereoVSNavigator*>( this ) );
//         traj_sub_ = nh_.subscribe(traj_topic, 10, &StereoVSNavigator::trajCB, this);
        ni_traj_sub_ = nh_.subscribe(ni_traj_topic, 10, &StereoVSNavigator::trajCB, dynamic_cast<StereoVSNavigator*>( this ));
        
        sub_l_img_.subscribe(nh_, "/multisense_sl/camera/left/image_raw", 1);
        sub_r_img_.subscribe(nh_, "/multisense_sl/camera/right/image_raw", 1);
        sub_l_info_.subscribe(nh_, "/multisense_sl/camera/left/camera_info", 1);
        sub_r_info_.subscribe(nh_, "/multisense_sl/camera/right/camera_info", 1);
        
        int queue_size;
        pnh_.param("queue_size", queue_size, 12);
        approximate_sync_img_.reset( new ApproximateSyncImage(ApproximatePolicyImage(queue_size),
                                                    sub_l_img_, sub_r_img_, sub_l_info_, sub_r_info_) );
        approximate_sync_img_->registerCallback(boost::bind(&StereoVSNoTrackingNavigator::imageCB,
                                                        this, _1, _2, _3, _4));
        
        odom_reset_pub_ = nh_.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);
        curr_local_traj_pub_ = nh_.advertise<nav_msgs::Path>(curr_local_traj_pub_topic, 1);
        
//         trajectory_publisher_ = nh_.advertise< nav_msgs::Path >("/vs/desired_trajectory", 10);
        
        command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 1);
        
        // Debug
        filtered_curr_features_img_ = nh_.advertise<sensor_msgs::Image>("/vs/filtered_curr_features_img", 1); 
        filtered_desired_features_img_ = nh_.advertise<sensor_msgs::Image>("/vs/filtered_desired_features_img", 1);
        next_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vs/next_goal", 1);
        raw_matching_img_pub_ = nh_.advertise<sensor_msgs::Image>("/vs/raw_matched_features", 1);
        
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

        vs_controller_ = boost::make_shared<trajectory_servo_no_tracking_core::VisualServoNoTrackingController>(tfBuffer_);
            
        if(!vs_controller_->initialized_)
        {
            vs_controller_->init();
            vs_controller_->setUncertaintyParams(sigmaMu, sigmaNu, sigmaD); // NOTE: No need
        }
        
        // if(!uncertainty_enabled && !feedforward_enabled && !speedcontrol_enabled)
        // {
        //     vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoController>(tfBuffer_);
            
        //     if(!vs_controller_->initialized_)
        //     {
        //         vs_controller_->init();
        //     }
        // }
        // else if(uncertainty_enabled && !feedforward_enabled && !speedcontrol_enabled)
        // {
        //     vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoControllerUncertainty>(tfBuffer_);
            
        //     if(!vs_controller_->initialized_)
        //     {
        //         vs_controller_->init();
        //         vs_controller_->setUncertaintyParams(sigmaMu, sigmaNu, sigmaD); // NOTE: No need
        //     }
        // }
        // else if(!uncertainty_enabled && feedforward_enabled && !speedcontrol_enabled)
        // {
        //     vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoControllerFeedforward>(tfBuffer_);
            
        //     if(!vs_controller_->initialized_)
        //     {
        //         vs_controller_->init();
        //     }
        // }
        // else if(uncertainty_enabled && feedforward_enabled && !speedcontrol_enabled)
        // {
        //     vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoControllerUncertaintyFeedforward>(tfBuffer_);
            
        //     if(!vs_controller_->initialized_)
        //     {
        //         vs_controller_->init();
        //         vs_controller_->setUncertaintyParams(sigmaMu, sigmaNu, sigmaD); // NOTE: No need
        //     }
        // }
        // else if(uncertainty_enabled && !feedforward_enabled && speedcontrol_enabled)
        // {
        //     vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoControllerUncertaintySpeedControl>(tfBuffer_, speedcontrol_p0);
            
        //     if(!vs_controller_->initialized_)
        //     {
        //         vs_controller_->init();
        //         vs_controller_->setUncertaintyParams(sigmaMu, sigmaNu, sigmaD); // NOTE: No need
        //     }
        // }
        // else if(uncertainty_enabled && feedforward_enabled && speedcontrol_enabled)
        // {
        //     vs_controller_ = boost::make_shared<trajectory_servo_core::VisualServoControllerUncertaintyFeedforwardSpeedControl>(tfBuffer_, speedcontrol_p0);
            
        //     if(!vs_controller_->initialized_)
        //     {
        //         vs_controller_->init();
        //         vs_controller_->setUncertaintyParams(sigmaMu, sigmaNu, sigmaD); // NOTE: No need
        //     }
        // }
        // else
        // {
        //     ROS_ERROR_STREAM("No valid controller for these configurations");
        // }
        
        
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
        
        
        return true;
    }
    
    
    void StereoVSNoTrackingNavigator::imageCB(const sensor_msgs::Image::ConstPtr& l_img,
                                              const sensor_msgs::Image::ConstPtr& r_img,
                                              const sensor_msgs::CameraInfo::ConstPtr& l_info,
                                              const sensor_msgs::CameraInfo::ConstPtr& r_info)
    {
        ROS_DEBUG_STREAM_NAMED(name_, "Features received.");
        ros::Time start_time = ros::Time::now();

        // Update stereo camera info
        stereo_camera_model_->fromCameraInfo(l_info, r_info);
        vs_controller_->registerStereoCameraModel(stereo_camera_model_);

        //Setup cam for feature matching
        cam_ = new PinholeStereoCamera(
                        l_info->width,
                        l_info->height,
                        fabs(l_info->P[0]),
                        fabs(l_info->P[5]),
                        l_info->P[2],
                        l_info->P[6],
                        r_info->P[3] / -r_info->P[0],
                        l_info->D[0],
                        l_info->D[1],
                        l_info->D[2],
                        l_info->D[3],
                        l_info->D[4] );

        ros::Time initialization_end_time = ros::Time::now();
        ROS_INFO_STREAM("Stereo initialization time: " << (initialization_end_time - start_time).toSec() * 1000 << " ms.");

        // variables for adaptative thresholds
        int orb_fast_th = Config::orbFastTh();
        double llength_th  = Config::minLineLength() * std::min( cam_->getWidth(), cam_->getHeight() ) ;

        // Features generation
        cv::Mat l_image = cv_bridge::toCvShare(l_img)->image;
        cv::Mat r_image = cv_bridge::toCvShare(r_img)->image;

        curr_frame_ =  new StereoFrame( l_image, r_image, 0, cam_ );

        ros::Time extraction_start_time = ros::Time::now();
        curr_frame_->extractStereoFeatures( llength_th, orb_fast_th );

        stereo_pt_ = curr_frame_->getStereoPt();
        std::vector< PointFeature* > stereo_pt_tmp = stereo_pt_;
        // ROS_INFO_STREAM(curr_frame_->stereo_pt.size());
        ros::Time extraction_end_time = ros::Time::now();
        ROS_INFO_STREAM("Feature extraction time: " << (extraction_end_time - extraction_start_time).toNSec() / 1000000 << " ms.");

        // Feature matching for servoing
        if(!traj_updated_ && frame_counter_ == 0)
        {
            if( Config::plInParallel())
            {
                auto detect_p = async(launch::async, &StereoVSNoTrackingNavigator::matchF2FPoints, this );
                detect_p.wait();
            }
            else
            {
                matchF2FPoints();
            }

            // ROS_INFO_STREAM(matched_pt_.size());
            stereo_pt_ = matched_pt_;
        }

        sparse_stereo_msgs::TrackedPointList::ConstPtr features = generateFeatureMsg(stereo_pt_, l_img->header);
        
        ROS_INFO_STREAM("Feature matching time: " << (ros::Time::now() - extraction_end_time).toSec() * 1000 << " ms.");

        // Debug
        if(features->tracked_list.size() <= 5)
        {
            cv::Mat vis_img_raw_matching(480, 640 * 2, CV_8UC3, cv::Scalar(255, 255, 255));

            for(auto pt : stereo_pt_tmp)
            {
                cv::circle(vis_img_raw_matching, cv::Point(pt->pl(0), pt->pl(1)), 4, cv::Scalar(0, 255, 0), 2);
            }

            for(auto pt : stereo_pt_)
            {
                cv::circle(vis_img_raw_matching, cv::Point(640 + pt->pl(0), pt->pl(1)), 4, cv::Scalar(255, 0, 0), 2);
            }

            cv_bridge::CvImage out_curr_msg;
            out_curr_msg.header   = l_img->header; // Same timestamp and tf frame as input image
            out_curr_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
            out_curr_msg.image    = vis_img_raw_matching; // Your cv::Mat
            raw_matching_img_pub_.publish(out_curr_msg.toImageMsg());

            // cv::imshow("tmp", vis_img_raw_matching);
            // cv::waitKey(0);
        }

        // Update traj if needed
        if(traj_updated_)
        {   
            frame_counter_ = 0;
            // Make a local copy of the local trajectory
            pips_trajectory_msgs::trajectory_points local_traj = getLocalNITrajLocalCopy();
            
            // Register local traj to the visual servo core
            ROS_INFO_STREAM_NAMED(name_, "New traj set. Initialization.");
            vs_controller_->initialization(local_traj);

            // Generate features
            first_frame_ = curr_frame_;
            // first_stereo_pt_ = stereo_pt_;
            
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
        }
        
        geometry_msgs::Twist cmd;
        if(!vs_controller_->goalReached())
        {
            ros::Time servo_start_time = ros::Time::now();

            if(continuous_type_)
            {
                if(ni_traj_enabled_)
                {
                    double pre_time_fraction;
                    vs_controller_->run_image(features, curr_odom_, pre_time_fraction, cmd);
                    if(vs_controller_->checkRegeneration(vs_controller_->getNumFilteredFeatures()))
                    {
                        first_frame_ = curr_frame_;
                        // first_stereo_pt_ = stereo_pt_tmp;
                        sparse_stereo_msgs::TrackedPointList::ConstPtr features_orig = generateFeatureMsg(stereo_pt_tmp, l_img->header);
                        vs_controller_->featureRegeneration(pre_time_fraction, features_orig, curr_odom_);
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
            command_publisher_.publish(cmd);
            ROS_INFO_STREAM("Servoing time: " << (ros::Time::now() - servo_start_time).toSec() * 1000 << " ms.");
            ROS_INFO_STREAM_NAMED(name_, "Command: " << cmd.linear.x <<"m/s, " << cmd.angular.z << "rad/s");
        }
        else
        {
            ROS_INFO_STREAM_NAMED(name_, "Goal reached.");
        }
        
        // Debug
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
                cv::Mat img_desired = drawFeaturesImg(desired_features, filtered_desired_features);
                cv_bridge::CvImage out_desired_msg;
                out_desired_msg.header   = features->header; // Same timestamp and tf frame as input image
                out_desired_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
                out_desired_msg.image    = img_desired; // Your cv::Mat
                filtered_desired_features_img_.publish(out_desired_msg.toImageMsg());
            }
        }
    }

    void StereoVSNoTrackingNavigator::matchF2FPoints()
    {

        // points f2f tracking
        // --------------------------------------------------------------------------------------------------------------------
        matched_pt_.clear();
        if ( !Config::hasPoints() || curr_frame_->stereo_pt.empty() || first_frame_->stereo_pt.empty() )
            return;

        std::vector<int> matches_12;
        match(first_frame_->pdesc_l, curr_frame_->pdesc_l, Config::minRatio12P(), matches_12);
        
        int num_tracked_iter = 0;
        // bucle around pmatches
        for (int i1 = 0; i1 < matches_12.size(); ++i1) {
            const int i2 = matches_12[i1];
            // ROS_INFO_STREAM("i2: " << i2);
            if (i2 < 0) continue;

            if (abs(curr_frame_->stereo_pt[i2]->pl(0) - first_frame_->stereo_pt[i1]->pl(0)) > 30 || 
                abs(curr_frame_->stereo_pt[i2]->pl(1) - first_frame_->stereo_pt[i1]->pl(1)) > 30)
                continue;
            
            PointFeature* point = first_frame_->stereo_pt[i1];
            point->pl_obs = curr_frame_->stereo_pt[i2]->pl;
            point->inlier = true;
            matched_pt_.push_back( point );
            curr_frame_->stereo_pt[i2]->idx = first_frame_->stereo_pt[i1]->idx; // prev idx
            
            // ROS_INFO_STREAM(point->idx);
            if(point->idx != -1)
            {
                num_tracked_iter++;
            }
        }
        
        int num_tracked_pt = num_tracked_iter;
    }
    
    sparse_stereo_msgs::TrackedPointList::ConstPtr StereoVSNoTrackingNavigator::generateFeatureMsg(const std::vector< PointFeature* >& stereo_pt, std_msgs::Header header)
    {
        sparse_stereo_msgs::TrackedPointList feature_msg;
        feature_msg.header = header;

        int start_idx = 0;
        for(auto pt : stereo_pt)
        {
            if(pt->idx > -1 && pt->P(2) >= 0.5 && pt->P(2) <= 10)
            {
                // pt->idx = start_idx;
                sparse_stereo_msgs::TrackedPoint tracked_pt_t;
                tracked_pt_t.header = header;

                tracked_pt_t.depth = pt->P(2);
                tracked_pt_t.id = pt->idx;
                tracked_pt_t.u_l = pt->pl(0);
                tracked_pt_t.v_l = pt->pl(1);

                feature_msg.tracked_list.push_back(tracked_pt_t);

                start_idx++;
            }
        }

        sparse_stereo_msgs::TrackedPointList::ConstPtr features = boost::make_shared<sparse_stereo_msgs::TrackedPointList const> (feature_msg);

        return features;
    }
}



