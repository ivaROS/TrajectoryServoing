#include <trajectory_servoing/trajectory_servo_feedforward.h>

namespace trajectory_servo_core
{
    VisualServoControllerFeedforward::VisualServoControllerFeedforward(tf_buffer_ptr tf_buffer) : 
        VisualServoController(tf_buffer)
    {
        
    }
    
    void VisualServoControllerFeedforward::init()
    {
        initialized_ = true;
        goal_reached_ = false;
        milestone_reached_ = false;
        
        vd_ = 0.05;
        lambda_ = 5;
        
        tx_ = 0.1;
        ty_ = 0;
        
        milestone_thresh_ = 0.1;
        final_goal_thresh_ =  milestone_thresh_;
        
        curr_index_ = -1;
        desired_index_ = -1;
        
        features_traj_.clear();
        retries_counter_ = 0;
        
        flags_modes_params_.goal_check_mode_ = USE_ODOM;

        // Uncertainty
        flags_modes_params_.uncertainty_enabled_ = false;
        
        // Feature trajectory regeneration
        flags_modes_params_.feat_traj_reg_enabled_ = true;
        flags_modes_params_.feat_traj_reg_mode_ = ODOM_POSE;
        flags_modes_params_.feat_traj_reg_thresh_ = 20;
        
        // Feature trajectory regeneration with mixing new and old features
        flags_modes_params_.feat_traj_reg_adding_enabled_ = false;
    }
    
    bool VisualServoControllerFeedforward::run_image(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom, double & pre_time_fraction, geometry_msgs::Twist& cmd, bool force_stop)
    {
        if(!initialized_ || !local_traj_registered_ || features->tracked_list.size() == 0)
        {
            if(force_stop)
                cmd = stop();
            curr_index_ = -1;
            desired_index_ = -1;
            
            if(local_traj_registered_ && features->tracked_list.size() == 0)
                num_filtered_pts_ = 0;

            return false;
        }
        
        // Note: Only use odom to check if the final goal is reached. Another method will reach to the final pose of traj
        bool goal_reached = false;
        switch (flags_modes_params_.goal_check_mode_)
        {
        case USE_ODOM:
            goal_reached = checkFinalGoalReached(odom);
            break;

        case LAST_POSE:
            goal_reached = checkFinalGoalReached();
            break;
        
        default:
            goal_reached = checkFinalGoalReached(odom);
            break;
        }

        if(goal_reached)
        {
            if(force_stop)
                cmd = stop();
            local_traj_registered_ = false;
            goal_reached_ = true;
            curr_index_ = -1;
            desired_index_ = -1;
            
            std::cout << "Average difference of the pose: " << dist / num_pose_diff_compute << " m." << std::endl;
            
            return false;
        }
        
        if(retries_counter_ >= 5)
        {
            if(force_stop)
                cmd = stop();
            local_traj_registered_ = false;
            curr_index_ = -1;
            desired_index_ = -1;
            
            std::cout << "The retries are over the max limits. Stop querying desired features." << std::endl;
            
            return false;
        }
        
        updateFeatures(features);
        std::cout << "Num of current features: " << current_features_.tracked_list.size() << std::endl;
        std::cout << "Current index of the feature trajectory: " << curr_index_ << std::endl;
        
        if(desired_index_ == -1)
        {
            traj_header_ = features->header;
        }
        
        FeaturesTraj desired_image;
        CurrFeatureTrajStats curr_feature_traj_stats;
        desiredVelocityCmd desired_vel = getDesiredFeatures(current_features_.header, desired_image, curr_feature_traj_stats);
        vd_ = float(desired_vel.vd_);
        wd_ = desired_vel.wd_;
        desired_index_ = curr_feature_traj_stats.curr_feature_traj_idx;
        double pre_time_fraction_tmp = curr_feature_traj_stats.pre_time_fraction;
        desired_features_ = desired_image.features_;
        tf2::Transform camInit2CamDes = desired_image.cam2cam_;
//         vd_ = desired_image.vd_;
        std::cout << "Desired velocity: [ " << vd_ << " , " << wd_ << " ]" << std::endl;
        
        // Debug
        ros::Duration elapsed_time = features->header.stamp - traj_header_.stamp;
        if((desired_image.time_ - elapsed_time) <= ros::Duration(0.05))
        {
            float x_diff = desired_image.x_ - odom->pose.pose.position.x;
            float y_diff = desired_image.y_ - odom->pose.pose.position.y;
            float pose_diff = sqrt(x_diff*x_diff+y_diff*y_diff);
            
//             tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
//             double roll, pitch, yaw;
//             tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            std::cout << "Pose difference: " << pose_diff << " m. x diff: " << x_diff << ", y diff: " << y_diff << std::endl;
            dist += pose_diff;
            num_pose_diff_compute++;
        }
        
        
        filterFeatures(desired_features_, current_features_, filtered_desired_features_, filtered_current_features_);
        
        assert(filtered_current_features_.tracked_list.size() == filtered_desired_features_.tracked_list.size());
        num_filtered_pts_ = filtered_current_features_.tracked_list.size();
        std::cout << "Num of features used: " << num_filtered_pts_ << " with pre time fraction: " << pre_time_fraction_tmp << std::endl;
        
        //Feature trajectory regeneration
//         if(flags_modes_params_.feat_traj_reg_enabled_ && !flags_modes_params_.feat_traj_reg_adding_enabled_)
//         {
//             if(num_filtered_pts_ <= flags_modes_params_.feat_traj_reg_thresh_)
//             {
//                 std::cout << "Feature trajectory regeneration..." << std::endl;
                
//                 switch(flags_modes_params_.feat_traj_reg_mode_)
//                 {
//                     case NAIVE_PERFECT_POSE:
//                         if((pre_time_fraction >= 0.9 || pre_time_fraction <= 0.1))
//                         {
//                             ros::Time start_fd = ros::Time::now();
                            
//                             std::vector<double> estimated_pose = featureAddingNaive(current_features_, features_traj_, desired_index_, pre_time_fraction);
// //                 
//                             // Debug
//                             float add_x_diff = estimated_pose[0] - odom->pose.pose.position.x;
//                             float add_y_diff = estimated_pose[1] - odom->pose.pose.position.y;
//                             float add_pose_diff = sqrt(add_x_diff*add_x_diff+add_y_diff*add_y_diff);
                            
//                             tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
//                             double roll, pitch, yaw;
//                             tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//                             double angle_diff = std::abs(yaw - estimated_pose[2]);
                            
//                             std::cout << "Pose difference between curr and estimated: position " << add_pose_diff << ", angle " << angle_diff / M_PI * 180 << std::endl;
                            
//                             ros::Time end_fd = ros::Time::now();
//                             ros::Duration d = end_fd - start_fd;
//                             traj_header_.stamp += d;
//                         }
                        
//                         break;
                        
//                     case ODOM_POSE or SLAM_POSE:
//                     {
//                         ros::Time start_fd = ros::Time::now();
//                         featureAddingNaiveGroundTruth(current_features_, features_traj_, desired_index_, pre_time_fraction, odom);
//                         ros::Time end_fd = ros::Time::now();
//                         ros::Duration d = end_fd - start_fd;
//                         traj_header_.stamp += d;
                        
//                         break;
//                     }
//                     case LOCAL_EST_POSE:
                        
//                         break;
                        
//                     default:
//                     {
//                         ros::Time start_fd = ros::Time::now();
//                         featureAddingNaiveGroundTruth(current_features_, features_traj_, desired_index_, pre_time_fraction, odom);
//                         ros::Time end_fd = ros::Time::now();
//                         ros::Duration d = end_fd - start_fd;
//                         traj_header_.stamp += d;
//                     }
//                 }
//             }
//         }

        pre_time_fraction = pre_time_fraction_tmp;
        
        // WARNING: may have issue here, if the num of features used suddenly drops to 0, feature adding is not performed, robot will stop.
        if(num_filtered_pts_ == 0)
        {
            if(force_stop)
                cmd = stop();
            retries_counter_++;
            std::cout << "The matched features are too low to support servoing." << std::endl;
            return false;
        }
        
        cmd = computeCmd(wd_);
        retries_counter_ = 0;
        return true;
    }

    geometry_msgs::Twist VisualServoControllerFeedforward::computeCmd(double wd)
    {
        cv::Mat image_jacobian, desired_image_jacobian;
        generateImageJacobian(image_jacobian, desired_image_jacobian);
        
        double trans[12] = {0, -tx_, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0};
        cv::Mat trans_mat = cv::Mat(6, 2, CV_64FC1, trans);
        
//         std::cout << "image jacobian rows: " << image_jacobian.rows << std::endl;
        assert(image_jacobian.cols == trans_mat.rows);
        
        cv::Mat image_jacobian_trans = image_jacobian * trans_mat;
        cv::Mat image_j_v = image_jacobian_trans.col(0).clone();
        cv::Mat image_j_w = image_jacobian_trans.col(1).clone();
        cv::Mat image_j_w_inv;
        cv::invert(image_j_w, image_j_w_inv, cv::DECOMP_SVD);

        cv::Mat desired_image_jacobian_trans = desired_image_jacobian * trans_mat;
        cv::Mat desired_image_j_v = desired_image_jacobian_trans.col(0).clone();
        cv::Mat desired_image_j_w = desired_image_jacobian_trans.col(1).clone();
        
        cv::Mat curr_feat_mat = featuresToMat(filtered_current_features_);
        cv::Mat desired_feat_mat = featuresToMat(filtered_desired_features_);
        
//         std::cout << "curr_mat: " << curr_feat_mat.rows << ", desired_mat: " << desired_feat_mat.rows << std::endl;
        
        assert(curr_feat_mat.rows == desired_feat_mat.rows);
        
        cv::Mat error = curr_feat_mat - desired_feat_mat;
        
//         std::cout << image_jacobian << std::endl;
        
        return controlLaw(error, image_j_w_inv, image_j_v, desired_image_j_v, desired_image_j_w);
    }

    void VisualServoControllerFeedforward::generateImageJacobian(cv::Mat& image_jacobian, cv::Mat& desired_image_jacobian)
    {
        int num_features = num_filtered_pts_;
        image_jacobian = cv::Mat::zeros(2*num_features, 6, CV_64FC1);
        desired_image_jacobian = cv::Mat::zeros(2*num_features, 6, CV_64FC1);
        
        for(size_t i = 0; i < num_features; i++)
        {
            sparse_stereo_msgs::TrackedPoint pt = filtered_current_features_.tracked_list[i];
            double u_l = ((double) pt.u_l) - cx_;
            double v_l = ((double) pt.v_l) - cy_;
//             float u_l = (float) pt.u_l;
//             float v_l = (float) pt.v_l;
            // std::cout << "ul: " << u_l << " vl: " << v_l << " depth: " << (double) pt.depth << std::endl;
            
            image_jacobian.at<double>(i*2, 0) = - fx_ / (double) pt.depth;
            image_jacobian.at<double>(i*2, 2) = u_l / (double) pt.depth;
            image_jacobian.at<double>(i*2, 3) = u_l * v_l / fx_;
            image_jacobian.at<double>(i*2, 4) = - (fx_ * fx_ + u_l * u_l) / fx_;
            image_jacobian.at<double>(i*2, 5) = v_l;
            
            image_jacobian.at<double>(i*2+1, 1) = - fy_ / (double) pt.depth;
            image_jacobian.at<double>(i*2+1, 2) = v_l / (double) pt.depth;
            image_jacobian.at<double>(i*2+1, 3) = (fy_ * fy_ + v_l * v_l) / fy_;
            image_jacobian.at<double>(i*2+1, 4) = - u_l * v_l / fy_;
            image_jacobian.at<double>(i*2+1, 5) = - u_l;

            // desired image jacobian
            sparse_stereo_msgs::TrackedPoint d_pt = filtered_desired_features_.tracked_list[i];
            double d_u_l = ((double) d_pt.u_l) - cx_;
            double d_v_l = ((double) d_pt.v_l) - cy_;
//             float u_l = (float) pt.u_l;
//             float v_l = (float) pt.v_l;
            // std::cout << "ul: " << u_l << " vl: " << v_l << " depth: " << (double) pt.depth << std::endl;
            
            desired_image_jacobian.at<double>(i*2, 0) = - fx_ / (double) d_pt.depth;
            desired_image_jacobian.at<double>(i*2, 2) = d_u_l / (double) d_pt.depth;
            desired_image_jacobian.at<double>(i*2, 3) = d_u_l * d_v_l / fx_;
            desired_image_jacobian.at<double>(i*2, 4) = - (fx_ * fx_ + d_u_l * d_u_l) / fx_;
            desired_image_jacobian.at<double>(i*2, 5) = d_v_l;
            
            desired_image_jacobian.at<double>(i*2+1, 1) = - fy_ / (double) d_pt.depth;
            desired_image_jacobian.at<double>(i*2+1, 2) = d_v_l / (double) d_pt.depth;
            desired_image_jacobian.at<double>(i*2+1, 3) = (fy_ * fy_ + d_v_l * d_v_l) / fy_;
            desired_image_jacobian.at<double>(i*2+1, 4) = - d_u_l * d_v_l / fy_;
            desired_image_jacobian.at<double>(i*2+1, 5) = - d_u_l;
        }
    }
    
    geometry_msgs::Twist VisualServoControllerFeedforward::controlLaw(const cv::Mat& error, const cv::Mat& j_w_inv, const cv::Mat& j_v, const cv::Mat& d_j_v, const cv::Mat& d_j_w)
    {
        cv::Mat ff = (double) vd_ * d_j_v + (double) wd_ * d_j_w;
        // std::cout << "Feedforward term: " << ff << std::endl;
        cv::Mat omega_mat = j_w_inv * (- (double) lambda_ * error - (double) vd_ * j_v + ff);
        assert(omega_mat.rows == 1);
        assert(omega_mat.cols == 1);
        double omega = omega_mat.at<double>(0, 0);
        
        geometry_msgs::Vector3 linear;
        linear.x = vd_;

        geometry_msgs::Vector3 angular;
        angular.z = omega;
        
        geometry_msgs::Twist cmd;
        cmd.linear = linear;
        cmd.angular = angular;
        
        return cmd;
    }
}
