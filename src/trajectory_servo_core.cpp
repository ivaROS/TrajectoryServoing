#include "trajectory_servoing/trajectory_servo_core.h"

namespace trajectory_servo_core
{
    VisualServoController::VisualServoController(tf_buffer_ptr tf_buffer) : 
    tfBuffer_(tf_buffer)
    {
        
    }
    
    VisualServoController::~VisualServoController()
    {
    }
    
    double VisualServoController::distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
    {
        double x_diff = p1.position.x - p2.position.x;
        double y_diff = p1.position.y - p2.position.y;
        double z_diff = p1.position.z - p2.position.z;
        
        return sqrt(x_diff*x_diff + y_diff*y_diff);
    }
    
    double VisualServoController::distance(const sparse_stereo_msgs::TrackedPoint& p1, sparse_stereo_msgs::TrackedPoint& p2)
    {
        double x_diff = (double) p1.u_l - (double) p2.u_l;
        double y_diff = (double) p1.v_l - (double) p2.v_l;
        
        return sqrt(x_diff*x_diff + y_diff*y_diff);
    }
    
    double VisualServoController::distance(const geometry_msgs::Pose& p1, pips_trajectory_msgs::trajectory_point& p2)
    {
        double x_diff = p1.position.x - p2.x;
        double y_diff = p1.position.y - p2.y;
        
        return sqrt(x_diff*x_diff + y_diff*y_diff);
    }
    
    void VisualServoController::init()
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
    
    void VisualServoController::reset()
    {
        // Remove all history
        removeLocalTraj();
        std_msgs::Header empty_header;
        current_features_.header = empty_header;
        current_features_.tracked_list.clear();
        desired_features_.header = empty_header;
        desired_features_.tracked_list.clear();
        
        filtered_current_features_ = current_features_;
        filtered_desired_features_ = desired_features_;
        
        goal_reached_ = false;
        milestone_reached_ = false;
        total_num_poses_ = 0;
        current_pose_idx_ = 0;
        
        num_filtered_pts_ = 0;
        curr_index_ = -1;
        desired_index_ = -1;
        retries_counter_ = 0;
        
        features_traj_.clear();
        
        // Pose diff
        dist = 0;
        num_pose_diff_compute = 0;
    }
    
    void VisualServoController::initialization(const nav_msgs::Path local_traj)
    {
        // Reset all history
        reset();
        
        // Register new local traj 
        registerLocalTraj(local_traj);
    }
    
    void VisualServoController::initialization(const pips_trajectory_msgs::trajectory_points local_traj)
    {
        // Reset all history
        reset();
        
        // Register new local traj 
        registerLocalTraj(local_traj);
    }
    
    void VisualServoController::removeLocalTraj()
    {
        local_traj_registered_ = false;
        std_msgs::Header empty_header;
        local_traj_.header = empty_header;
        local_traj_.poses.clear();
        local_ni_traj_.header = empty_header;
        local_ni_traj_.points.clear();
    }
    
    void VisualServoController::registerCameraBaseTrans(const geometry_msgs::TransformStamped& geometry_trans)
    {
        tf2::Vector3 trans(geometry_trans.transform.translation.x, geometry_trans.transform.translation.y, geometry_trans.transform.translation.z);
        tf2::Quaternion quat(geometry_trans.transform.rotation.x, geometry_trans.transform.rotation.y, geometry_trans.transform.rotation.z, geometry_trans.transform.rotation.w);
        
        camera_base_trans_.setOrigin(trans);
        camera_base_trans_.setRotation(quat);
    }
    
    void VisualServoController::registerStereoCameraModel(const boost::shared_ptr<image_geometry::StereoCameraModel>& stereo)
    {
        stereo_camera_model_ = stereo;
        fx_ = stereo_camera_model_->left().fx();
        fy_ = stereo_camera_model_->left().fy();
        cx_ = stereo_camera_model_->left().cx();
        cy_ = stereo_camera_model_->left().cy();
        image_width_ = 2*floor(stereo_camera_model_->left().cx());
        image_height_ = 2*floor(stereo_camera_model_->left().cy());
        cameraMat_ = cv::Mat(stereo_camera_model_->left().intrinsicMatrix());
        cameraMatInv_ = cameraMat_.inv();
        baseline_ = stereo_camera_model_->baseline();
    }
    
    void VisualServoController::registerLocalTraj(const nav_msgs::Path local_traj)
    {
        if(!local_traj_registered_)
        {
            boost::mutex::scoped_lock lock(core_mutex_);
            local_traj_ = local_traj;
            local_traj_registered_ = true;
            
            total_num_poses_ = local_traj_.poses.size();
            current_pose_idx_ = 0;
            
        }
    }
    
    void VisualServoController::registerLocalTraj(const pips_trajectory_msgs::trajectory_points local_traj)
    {
        if(!local_traj_registered_)
        {
            boost::mutex::scoped_lock lock(core_mutex_);
            local_ni_traj_ = local_traj;
            local_traj_registered_ = true;
            retries_counter_ = 0;
            
            total_num_poses_ = local_ni_traj_.points.size();
            current_pose_idx_ = 0;
            
        }
    }
    
    void VisualServoController::updateNextGoal()
    {
        current_pose_idx_ += 1;
        if(current_pose_idx_ >= total_num_poses_)
        {
            std::cout << "Final goal already set as the next goal." << std::endl;
//             return;
            current_pose_idx_ = total_num_poses_ - 1;
        }
        
        next_goal_ = local_traj_.poses[current_pose_idx_];
        milestone_reached_ = false;
    }
    
    bool VisualServoController::checkNextGoalReached(const nav_msgs::Odometry::ConstPtr& odom)
    {
        double dis = distance(odom->pose.pose, next_goal_.pose);
        std::cout << "Distance to the milestone: " << dis << std::endl;
        if(dis <= milestone_thresh_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    bool VisualServoController::checkFinalGoalReached(const nav_msgs::Odometry::ConstPtr& odom)
    {
        double dis;
        if(local_traj_.poses.size() != 0)
        {
            dis = distance(odom->pose.pose, local_traj_.poses.back().pose);
        }
        else if(local_ni_traj_.points.size() != 0)
        {
            dis = distance(odom->pose.pose, local_ni_traj_.points.back());
        }
        else
        {
            ROS_WARN_STREAM("Local traj size is 0.");
        }
        
        if(dis <= final_goal_thresh_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool VisualServoController::checkFinalGoalReached()
    {
        if(local_traj_registered_ && features_traj_.size() > 0 && desired_index_ == features_traj_.size() - 1 )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    void VisualServoController::generateAllFeatures(const sparse_stereo_msgs::TrackedPointList::ConstPtr& current_features, 
                                                    const pips_trajectory_msgs::trajectory_points& traj, const nav_msgs::Odometry::ConstPtr& odom)
    {
//         traj_header_ = traj.header;
        features_traj_.clear();
        
        // push back current features for the first
//         FeaturesTraj zero_features(*current_features, traj.points[0].time, traj.points[0].v);
//         features_traj_.push_back(zero_features);

        // Current transform based on the current odom
        tf2::Quaternion curr_quat;
        tf2::fromMsg(odom->pose.pose.orientation, curr_quat);
        tf2::Vector3 curr_trans(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
        tf2::Transform curr_transformation(curr_quat, curr_trans);
        
        for(size_t i = 1; i < traj.points.size(); i++)
        {
            FeaturesTraj desired_features;
            desired_features.frame_id_ = odom->header.frame_id;
            
            generateDesiredFeatures(current_features, traj.points[i], desired_features, curr_transformation);
            
//             if(desired_features.features_.tracked_list.size() == 0)
//                 break;
            
            features_traj_.push_back(desired_features);
        }
        
//         std::cout << "Computed [ " << features_traj_.size() << " ] desired images." << std::endl;
        std::cout << features_traj_.size() << " " << traj.points.size() << std::endl;
    }
        
    void VisualServoController::generateDesiredFeatures(const sparse_stereo_msgs::TrackedPointList::ConstPtr& current_features, 
                                                        const pips_trajectory_msgs::trajectory_point& traj_pose, 
                                                        FeaturesTraj& feature_traj, tf2::Transform curr_trans)
    {
        int num_curr_features = current_features->tracked_list.size();
        std::cout << "Num of features used to generate desired: " << num_curr_features << std::endl;
        
//         // Uncertainty
//         const double mean = 0.0;
//         const double stddev = 0;
//         std::default_random_engine generator;
//         std::normal_distribution<double> dist(mean, stddev);
//         ROS_INFO_STREAM(dist(generator));
        
        sparse_stereo_msgs::TrackedPointList desired_features;
        desired_features.header = current_features->header;
        
        tf2::Quaternion desired_quat;
        desired_quat.setRPY(0, 0, traj_pose.theta);
        tf2::Vector3 desired_trans(traj_pose.x, traj_pose.y, 0);
        tf2::Transform desired_transformation(desired_quat, desired_trans);
        
        tf2::Transform cam2CamDes = camera_base_trans_.inverse() * desired_transformation.inverse() * curr_trans * camera_base_trans_;
        
        for(size_t i = 0; i < num_curr_features; i++)
        {
            sparse_stereo_msgs::TrackedPoint pt = current_features->tracked_list[i];
            // Reproject the image features to the 3D
            cv::Point3d pt_3d;
//             cv::Point2d pt_img(pt.u_l + dist(generator), pt.v_l + dist(generator));
            cv::Point2d pt_img(pt.u_l, pt.v_l);
//             double disparity = fx_ * baseline_ / pt.depth;
//             disparity = disparity + dist(generator);
//             pt.depth = fx_ * baseline_ / disparity;
            
            pt_3d = stereo_camera_model_->left().projectPixelTo3dRay(pt_img);
            // double disparity = fx_ * baseline_ / pt.depth;
            // pt.depth=pt.depth*(1.0+2.0*pt.var/(disparity*disparity));
            pt_3d = pt.depth * pt_3d;
            
            // std::cout<<"disparity"<<disparity<<std::endl;
            // std::cout<<"pt.u_l"<<pt.u_l<<std::endl;
            // std::cout<<"pt.v_l"<<pt.v_l<<std::endl;
            // std::cout<<"pt.var"<<pt.var<<std::endl;
            tf2::Vector3 pt_curr(pt_3d.x, pt_3d.y, pt_3d.z);
            
            // Transform to the desired pose
            
//             tf2::Quaternion curr_quat(0, 0, 0, 1);
//             tf2::Vector3 curr_trans(0, 0, 0);
//             tf2::Transform curr_transformation(curr_quat, curr_trans);
            
            tf2::Vector3 pt_desired = cam2CamDes * pt_curr;
            cv::Point3d pt_3d_desired(pt_desired.getX(), pt_desired.getY(), pt_desired.getZ());
            
            if(pt_3d_desired.z > 0)
            {
                // Project into the image and check if it is visible
                cv::Point2d pt_desired_img = stereo_camera_model_->left().project3dToPixel(pt_3d_desired);
                if(pt_desired_img.x >= 0 && pt_desired_img.x <= image_width_-1 && pt_desired_img.y >= 0 && pt_desired_img.y <= image_height_-1)
                {
                    sparse_stereo_msgs::TrackedPoint desired_pt;
                    desired_pt.header = pt.header;
                    desired_pt.u_l = (float) (pt_desired_img.x);
                    desired_pt.v_l = (float) (pt_desired_img.y);
                    desired_pt.depth = (float) pt_3d_desired.z;
                    desired_pt.u_l_init = (float) (pt.u_l);
                    desired_pt.v_l_init = (float) (pt.v_l);
                    desired_pt.x_des = (float) pt_3d_desired.x;
                    desired_pt.y_des = (float) pt_3d_desired.y;
                    desired_pt.depth_init = (float) pt.depth;
                    desired_pt.id = pt.id;
                    desired_pt.var = pt.var;
                    desired_features.tracked_list.push_back(desired_pt);
                }
            }
        }
        
        if(desired_features.tracked_list.size() == 0)
        {
            feature_traj.valid_ = false;
        }
        else
        {
            feature_traj.valid_ = true;
        }
        
        feature_traj.features_ = desired_features;
        feature_traj.time_ = traj_pose.time;
        feature_traj.vd_ = traj_pose.v;
        feature_traj.wd_ = traj_pose.w;
        feature_traj.x_ = traj_pose.x;
        feature_traj.y_ = traj_pose.y;
        feature_traj.theta_ = traj_pose.theta;
        feature_traj.cam2cam_ = cam2CamDes;
        
        std::cout << "Desired pose time: " << feature_traj.time_ << " with [ " << desired_features.tracked_list.size() << " ] features." << std::endl;
    }
    
    void VisualServoController::generateDesiredFeatures(const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::PoseStamped& goal_pose)
    {
        desired_features_.header = current_features_.header;
        desired_features_.tracked_list.clear();
        filtered_desired_features_.header = desired_features_.header;
        filtered_desired_features_.tracked_list.clear();
        filtered_current_features_.header = current_features_.header;
        filtered_current_features_.tracked_list.clear();
        
        int num_curr_features = current_features_.tracked_list.size();
        std::cout << "Num of features used to generate desired: " << num_curr_features << std::endl;
        for(size_t i = 0; i < num_curr_features; i++)
        {
            sparse_stereo_msgs::TrackedPoint pt = current_features_.tracked_list[i];
            // Reproject the image features to the 3D
            cv::Point3d pt_3d;
            cv::Point2d pt_img(pt.u_l, pt.v_l);
            pt_3d = stereo_camera_model_->left().projectPixelTo3dRay(pt_img);
            pt_3d = pt.depth * pt_3d;
            tf2::Vector3 pt_curr(pt_3d.x, pt_3d.y, pt_3d.z);
            
            // Transform to the desired pose
            tf2::Quaternion curr_quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            tf2::Vector3 curr_trans(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
            tf2::Transform curr_transformation(curr_quat, curr_trans);
            tf2::Quaternion desired_quat(goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
            tf2::Vector3 desired_trans(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
            tf2::Transform desired_transformation(desired_quat, desired_trans);
            
            tf2::Vector3 pt_desired = camera_base_trans_.inverse() * desired_transformation.inverse() * curr_transformation * camera_base_trans_ * pt_curr;
            cv::Point3d pt_3d_desired(pt_desired.getX(), pt_desired.getY(), pt_desired.getZ());
            
            // Project into the image and check if it is visible
            cv::Point2d pt_desired_img = stereo_camera_model_->left().project3dToPixel(pt_3d_desired);
            if(pt_desired_img.x >= 0 && pt_desired_img.x <= image_width_-1 && pt_desired_img.y >= 0 && pt_desired_img.y <= image_height_-1)
            {
                sparse_stereo_msgs::TrackedPoint desired_pt;
                desired_pt.header = pt.header;
                desired_pt.u_l = (int) std::round(pt_desired_img.x);
                desired_pt.v_l = (int) std::round(pt_desired_img.y);
                desired_pt.depth = (float) pt_3d_desired.z;
                desired_pt.id = pt.id;
                desired_features_.tracked_list.push_back(desired_pt);
                
                filtered_current_features_.tracked_list.push_back(pt);
            }
        }
        filtered_desired_features_ = desired_features_;  
    }
    
    // double VisualServoController::getDesiredFeatures(const std_msgs::Header& current_header, FeaturesTraj& post_features, 
    //                                                  CurrFeatureTrajStats& curr_feature_traj_stats)
    // {
    //     sparse_stereo_msgs::TrackedPointList desired_features;
    //     desired_features.header = current_header;
    //     double pre_time_fraction = 1;
        
    //     int num_points = features_traj_.size();
        
    //     ros::Duration elapsed_time = current_header.stamp - traj_header_.stamp;
    //     double t = elapsed_time.toSec();
    //     std::cout << "The elapsed time: " << t << " s." << std::endl;
        
    //     for(; curr_index_ < num_points -1 && features_traj_[curr_index_+1].time_ < elapsed_time; curr_index_++);
        
    //     int post_index = std::min(curr_index_+1, num_points-1);
        
    //     std::cout << "The desired image index: " << post_index << std::endl;
        
    //     post_features = features_traj_[post_index];
    //     FeaturesTraj pre_features;
    //     ros::Duration pre_time, period;
    //     double interpolated_vd;
        
    //     if(post_index == 0)
    //     {
    //         interpolated_vd = post_features.vd_;
    //     }
    //     else
    //     {
    //         pre_features = features_traj_[curr_index_];
        
    //         pre_time = elapsed_time - pre_features.time_;
    //         period = post_features.time_ - pre_features.time_;
        
    //         if(curr_index_ < features_traj_.size()-1)
    //         {
    //             pre_time_fraction = pre_time.toSec()/period.toSec();
    //         }
            
    //         interpolated_vd = pre_features.vd_*(1-pre_time_fraction) + post_features.vd_*pre_time_fraction;
    //     }
        
    //     if(curr_index_ == num_points - 1)
    //     {
    //         curr_index_ = -1;
    //     }
        
    //     curr_feature_traj_stats.curr_feature_traj_idx = post_index;
    //     curr_feature_traj_stats.pre_time_fraction = pre_time_fraction;
        
    //     return interpolated_vd;
    // }

    desiredVelocityCmd VisualServoController::getDesiredFeatures(const std_msgs::Header& current_header, FeaturesTraj& post_features, 
                                                     CurrFeatureTrajStats& curr_feature_traj_stats)
    {
        sparse_stereo_msgs::TrackedPointList desired_features;
        desired_features.header = current_header;
        double pre_time_fraction = 1;
        
        int num_points = features_traj_.size();
        
        ros::Duration elapsed_time = current_header.stamp - traj_header_.stamp;
        double t = elapsed_time.toSec();
        std::cout << "The elapsed time: " << t << " s." << std::endl;
        
        for(; curr_index_ < num_points -1 && features_traj_[curr_index_+1].time_ < elapsed_time; curr_index_++);
        
        int post_index = std::min(curr_index_+1, num_points-1);
        
        std::cout << "The desired image index: " << post_index << std::endl;
        
        post_features = features_traj_[post_index];
        FeaturesTraj pre_features;
        ros::Duration pre_time, period;
        double interpolated_vd, interpolated_wd;
        
        if(post_index == 0)
        {
            interpolated_vd = post_features.vd_;
            interpolated_wd = post_features.wd_;
        }
        else
        {
            pre_features = features_traj_[curr_index_];
        
            pre_time = elapsed_time - pre_features.time_;
            period = post_features.time_ - pre_features.time_;
        
            if(curr_index_ < features_traj_.size()-1)
            {
                pre_time_fraction = pre_time.toSec()/period.toSec();
            }
            
            interpolated_vd = pre_features.vd_*(1-pre_time_fraction) + post_features.vd_*pre_time_fraction;
            interpolated_wd = pre_features.wd_*(1-pre_time_fraction) + post_features.wd_*pre_time_fraction;
        }
        
        if(curr_index_ == num_points - 1)
        {
            curr_index_ = -1;
        }
        
        curr_feature_traj_stats.curr_feature_traj_idx = post_index;
        curr_feature_traj_stats.pre_time_fraction = pre_time_fraction;

        desiredVelocityCmd out_vel(interpolated_vd, interpolated_wd);
        
        return out_vel;
    }
    
    void VisualServoController::filterFeatures(const sparse_stereo_msgs::TrackedPointList& f1, const sparse_stereo_msgs::TrackedPointList& f2,
                            sparse_stereo_msgs::TrackedPointList& f1_out, sparse_stereo_msgs::TrackedPointList& f2_out)
    {
        f1_out.header = f1.header;
        f1_out.tracked_list.clear();
        f2_out.header = f2.header;
        f2_out.tracked_list.clear();
        int num_f1_features = f1.tracked_list.size();
        int num_f2_features = f2.tracked_list.size();
        
        for(size_t i = 0; i < num_f1_features; i++)
        {
            sparse_stereo_msgs::TrackedPoint pt1 = f1.tracked_list[i];
            for(size_t j = 0; j < num_f2_features; j++)
            {
                sparse_stereo_msgs::TrackedPoint pt2 = f2.tracked_list[j];
                // if(pt1.id == pt2.id && abs(pt1.u_l - pt2.u_l) <= 60 && abs(pt1.v_l - pt2.v_l) <= 60)
                if(pt1.id == pt2.id)
                {
                    f1_out.tracked_list.push_back(pt1);
                    f2_out.tracked_list.push_back(pt2);
                }
            }
        }
//         std::cout << num_f1_features << " " << num_f2_features << std::endl;
//         int num_repeat = 0;
//         for(size_t i = 0; i < num_f2_features; i++)
//         {
//             sparse_stereo_msgs::TrackedPoint pt1 = f2.tracked_list[i];
//             for(size_t j = 0; j < num_f2_features; j++)
//             {
//                 sparse_stereo_msgs::TrackedPoint pt2 = f2.tracked_list[j];
//                 if(pt1.id == pt2.id && i != j)
//                 {
//                     std::cout << pt1.id << std::endl;
//                     num_repeat++;
//                 }
//             }
//         }
//         std::cout << num_repeat << std::endl;
    }
    
    void VisualServoController::featureAddingNaiveGroundTruth(const sparse_stereo_msgs::TrackedPointList& current_features, 
                                                              std::vector<FeaturesTraj>& features_traj, 
                                                              int curr_feature_traj_idx, double pre_time_fraction,
                                                              const nav_msgs::Odometry::ConstPtr& odom)
    {
        int num_curr_features = current_features.tracked_list.size();
//         std::cout << "Num of features used to generate desired: " << num_curr_features << std::endl;
//         FeaturesTraj curr_desired_feature_traj = features_traj[curr_feature_traj_idx];
        
        double estimated_curr_x, estimated_curr_y, estimated_curr_theta;
//         FeaturesTraj post_point = features_traj[curr_feature_traj_idx];
//         if(curr_index_ == -1 && pre_time_fraction == 1)
//         {
//             estimated_curr_x = post_point.x_;
//             estimated_curr_y = post_point.y_;
//             estimated_curr_theta = post_point.theta_;
//         }
//         else
//         {
//             FeaturesTraj pre_point = features_traj[curr_index_];
//             
//             estimated_curr_x = pre_point.x_*(1-pre_time_fraction) + post_point.x_*pre_time_fraction;
//             estimated_curr_y = pre_point.y_*(1-pre_time_fraction) + post_point.y_*pre_time_fraction;
//             estimated_curr_theta = pre_point.theta_*(1-pre_time_fraction) + post_point.theta_*pre_time_fraction;
//         }
        
        estimated_curr_x = odom->pose.pose.position.x;
        estimated_curr_y = odom->pose.pose.position.y;
        
        tf2::Quaternion curr_quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf2::Vector3 curr_trans(estimated_curr_x, estimated_curr_y, 0);
        tf2::Transform curr_transformation(curr_quat, curr_trans);
        
//         std::cout << "Odom: ( " << estimated_curr_x << " , " << estimated_curr_y << " )." << std::endl;
        
        unsigned long max_future_pose = 20;
        
        for(size_t i = curr_feature_traj_idx; i < std::min(features_traj.size(), curr_feature_traj_idx + max_future_pose); i++)
        {
            FeaturesTraj curr_desired_feature_traj = features_traj[i];
            sparse_stereo_msgs::TrackedPointList old_desired_features = curr_desired_feature_traj.features_;
            
            sparse_stereo_msgs::TrackedPointList new_desired_features;
//             new_desired_features = old_desired_features;
            new_desired_features.header = current_features.header;

            tf2::Quaternion desired_quat;
            desired_quat.setRPY(0, 0, curr_desired_feature_traj.theta_);
            tf2::Vector3 desired_trans(curr_desired_feature_traj.x_, curr_desired_feature_traj.y_, 0);
            tf2::Transform desired_transformation(desired_quat, desired_trans);
            
            tf2::Transform cam2CamDes = camera_base_trans_.inverse() * desired_transformation.inverse() * curr_transformation * camera_base_trans_;
            
//             std::cout << "Desired pose: ( " << curr_desired_feature_traj.x_ << " , " << curr_desired_feature_traj.y_ << " , " << curr_desired_feature_traj.theta_ << " )." << std::endl;
            
            for(size_t j = 0; j < num_curr_features; j++)
            {
                sparse_stereo_msgs::TrackedPoint pt = current_features.tracked_list[j];
                
                // Reproject the image features to the 3D
                cv::Point3d pt_3d;
                cv::Point2d pt_img(pt.u_l, pt.v_l);
                pt_3d = stereo_camera_model_->left().projectPixelTo3dRay(pt_img);
                pt_3d = pt.depth * pt_3d;
                tf2::Vector3 pt_curr(pt_3d.x, pt_3d.y, pt_3d.z);
                
//                 std::cout << "Curr features in the curr: ( " << pt_curr.getX() << " , " << pt_curr.getY() << " , " << pt_curr.getZ() << " )." << std::endl;
                // Transform to the desired pose
//                 tf2::Quaternion curr_quat;
//                 curr_quat.setRPY(0, 0, estimated_curr_theta);

                tf2::Vector3 pt_curr_odom = curr_transformation * camera_base_trans_ * pt_curr;
//                 std::cout << "Curr features in the odom: ( " << pt_curr_odom.getX() << " , " << pt_curr_odom.getY() << " , " << pt_curr_odom.getZ() << " )." << std::endl;
                
                tf2::Vector3 pt_curr_desired = desired_transformation.inverse() * pt_curr_odom;
                tf2::Vector3 pt_desired = camera_base_trans_.inverse() * pt_curr_desired;
                cv::Point3d pt_3d_desired(pt_desired.getX(), pt_desired.getY(), pt_desired.getZ());
                
//                 std::cout << "Curr features in the desired: ( " << pt_curr_desired.getX() << " , " << pt_curr_desired.getY() << " , " << pt_curr_desired.getZ() << " )." << std::endl;
                
                if(pt_3d_desired.z > 0)
                {
                    // Project into the image and check if it is visible
                    cv::Point2d pt_desired_img = stereo_camera_model_->left().project3dToPixel(pt_3d_desired);
                    if(pt_desired_img.x >= 0 && pt_desired_img.x <= image_width_-1 && pt_desired_img.y >= 0 && pt_desired_img.y <= image_height_-1)
                    {
                        sparse_stereo_msgs::TrackedPoint desired_pt;
                        desired_pt.header = pt.header;
                        desired_pt.u_l = (float) std::round(pt_desired_img.x);
                        desired_pt.v_l = (float) std::round(pt_desired_img.y);
                        desired_pt.depth = (float) pt_3d_desired.z;
                        desired_pt.u_l_init = (float) (pt.u_l);
                        desired_pt.v_l_init = (float) (pt.v_l);
                        desired_pt.x_des = (float) pt_3d_desired.x;
                        desired_pt.y_des = (float) pt_3d_desired.y;
                        desired_pt.depth_init = (float) pt.depth;
                        desired_pt.id = pt.id;
                        
//                         std::cout << desired_pt.u_l << " " << desired_pt.v_l << std::endl;
                        // Check if the pt is existed
//                         bool repeated = false;
//                         for(size_t k = 0; k < old_desired_features.tracked_list.size(); k++)
//                         {
//                             double dist = distance(desired_pt, old_desired_features.tracked_list[k]);
// //                             std::cout << old_desired_features.tracked_list[k].u_l << " " << old_desired_features.tracked_list[k].v_l << " " << dist << std::endl;
//                             if(pt.id == old_desired_features.tracked_list[k].id || dist < 20)
//                             {
//                                 repeated = true;
//                                 break;
//                             }
//                         }
//                         if(!repeated)
//                         {
//                             new_desired_features.tracked_list.push_back(desired_pt);
//                         }
                        
                        new_desired_features.tracked_list.push_back(desired_pt);
                    }
                }
            }
            
//             std::cout << "current features: " << num_curr_features << ", old features size: " << old_desired_features.tracked_list.size() << ", new desired features size: " <<  new_desired_features.tracked_list.size() << std::endl;
//             if(i == curr_feature_traj_idx)
//             {
//                 for(size_t a = 0; a < new_desired_features.tracked_list.size();a++)
//                 {
//                     std::cout << new_desired_features.tracked_list[a].u_l << " " << new_desired_features.tracked_list[a].v_l << " " << new_desired_features.tracked_list[a].id << std::endl;
//                 }
//             }
            if(new_desired_features.tracked_list.size() == 0)
            {
                features_traj[i].valid_ = false;
            }
            else
            {
                features_traj[i].valid_ = true;
            }
            features_traj[i].features_ = new_desired_features;
            features_traj[i].cam2cam_ = cam2CamDes;
        }
    }
    
    std::vector<double> VisualServoController::featureAddingNaive(const sparse_stereo_msgs::TrackedPointList& current_features, 
                                              std::vector<FeaturesTraj>& features_traj, int curr_feature_traj_idx, double pre_time_fraction)
    {
        int num_curr_features = current_features.tracked_list.size();
//         std::cout << "Num of features used to generate desired: " << num_curr_features << std::endl;
//         FeaturesTraj curr_desired_feature_traj = features_traj[curr_feature_traj_idx];
        
        double estimated_curr_x, estimated_curr_y, estimated_curr_theta;
        FeaturesTraj post_point = features_traj[curr_feature_traj_idx];
        if(curr_index_ == -1 && pre_time_fraction == 1)
        {
            estimated_curr_x = post_point.x_;
            estimated_curr_y = post_point.y_;
            estimated_curr_theta = post_point.theta_;
        }
        else
        {
            FeaturesTraj pre_point = features_traj[curr_index_];
            
            estimated_curr_x = pre_point.x_*(1-pre_time_fraction) + post_point.x_*pre_time_fraction;
            estimated_curr_y = pre_point.y_*(1-pre_time_fraction) + post_point.y_*pre_time_fraction;
            estimated_curr_theta = pre_point.theta_*(1-pre_time_fraction) + post_point.theta_*pre_time_fraction;
        }
        
        std::vector<double> estimated_pose{estimated_curr_x, estimated_curr_y, estimated_curr_theta};
        
        tf2::Quaternion curr_quat;
        curr_quat.setRPY(0, 0, estimated_curr_theta);
        tf2::Vector3 curr_trans(estimated_curr_x, estimated_curr_y, 0);
        tf2::Transform curr_transformation(curr_quat, curr_trans);
        
        for(size_t i = curr_feature_traj_idx; i < features_traj.size(); i++)
        {
            FeaturesTraj curr_desired_feature_traj = features_traj[i];
            sparse_stereo_msgs::TrackedPointList old_desired_features = curr_desired_feature_traj.features_;
            
            sparse_stereo_msgs::TrackedPointList new_desired_features;
//             new_desired_features = old_desired_features;
            new_desired_features.header = current_features.header;
            
            tf2::Quaternion desired_quat;
            desired_quat.setRPY(0, 0, curr_desired_feature_traj.theta_);
            tf2::Vector3 desired_trans(curr_desired_feature_traj.x_, curr_desired_feature_traj.y_, 0);
            tf2::Transform desired_transformation(desired_quat, desired_trans);
            
            for(size_t j = 0; j < num_curr_features; j++)
            {
                sparse_stereo_msgs::TrackedPoint pt = current_features.tracked_list[j];
                
                // Reproject the image features to the 3D
                cv::Point3d pt_3d;
                cv::Point2d pt_img(pt.u_l, pt.v_l);
                pt_3d = stereo_camera_model_->left().projectPixelTo3dRay(pt_img);
                pt_3d = pt.depth * pt_3d;
                tf2::Vector3 pt_curr(pt_3d.x, pt_3d.y, pt_3d.z);
                
                // Transform to the desired pose
                tf2::Vector3 pt_desired = camera_base_trans_.inverse() * desired_transformation.inverse() * curr_transformation * camera_base_trans_ * pt_curr;
                cv::Point3d pt_3d_desired(pt_desired.getX(), pt_desired.getY(), pt_desired.getZ());
                
                if(pt_3d_desired.z > 0)
                {
                    // Project into the image and check if it is visible
                    cv::Point2d pt_desired_img = stereo_camera_model_->left().project3dToPixel(pt_3d_desired);
                    if(pt_desired_img.x >= 0 && pt_desired_img.x <= image_width_-1 && pt_desired_img.y >= 0 && pt_desired_img.y <= image_height_-1)
                    {
                        sparse_stereo_msgs::TrackedPoint desired_pt;
                        desired_pt.header = pt.header;
                        desired_pt.u_l = (int) std::round(pt_desired_img.x);
                        desired_pt.v_l = (int) std::round(pt_desired_img.y);
                        desired_pt.depth = (float) pt_3d_desired.z;
                        desired_pt.id = pt.id;
                        
//                         std::cout << desired_pt.u_l << " " << desired_pt.v_l << std::endl;
                        // Check if the pt is existed
//                         bool repeated = false;
//                         for(size_t k = 0; k < old_desired_features.tracked_list.size(); k++)
//                         {
//                             double dist = distance(desired_pt, old_desired_features.tracked_list[k]);
// //                             std::cout << old_desired_features.tracked_list[k].u_l << " " << old_desired_features.tracked_list[k].v_l << " " << dist << std::endl;
//                             if(pt.id == old_desired_features.tracked_list[k].id || dist < 20)
//                             {
//                                 repeated = true;
//                                 break;
//                             }
//                         }
//                         if(!repeated)
//                         {
//                             new_desired_features.tracked_list.push_back(desired_pt);
//                         }
                        new_desired_features.tracked_list.push_back(desired_pt);
                    }
                }
            }
            
//             std::cout << "current features: " << num_curr_features << " old features size: " << old_desired_features.tracked_list.size() << " new desired features size: " <<  new_desired_features.tracked_list.size() << std::endl;
//             if(i == curr_feature_traj_idx)
//             {
//                 for(size_t a = 0; a < new_desired_features.tracked_list.size();a++)
//                 {
//                     std::cout << new_desired_features.tracked_list[a].u_l << " " << new_desired_features.tracked_list[a].v_l << " " << new_desired_features.tracked_list[a].id << std::endl;
//                 }
//             }
            
            features_traj[i].features_ = new_desired_features;
        }
//         sleep(5);

        return estimated_pose;
    }

    bool VisualServoController::checkRegeneration(int num_filtered_pts)
    {
        if(flags_modes_params_.feat_traj_reg_enabled_ && !flags_modes_params_.feat_traj_reg_adding_enabled_)
        {
            if(initialized_ && local_traj_registered_ && num_filtered_pts <= flags_modes_params_.feat_traj_reg_thresh_)
            {
                return true;
            }
        }
        return false;
    }

    void VisualServoController::featureRegeneration(double pre_time_fraction, const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom)
    {
        if(!initialized_ || !local_traj_registered_ || features->tracked_list.size() == 0)
        {
            return;
        }
        std::cout << "Feature trajectory regeneration..." << std::endl;
                
        switch(flags_modes_params_.feat_traj_reg_mode_)
        {
            case NAIVE_PERFECT_POSE:
                if((pre_time_fraction >= 0.9 || pre_time_fraction <= 0.1))
                {
                    ros::Time start_fd = ros::Time::now();
                    
                    std::vector<double> estimated_pose = featureAddingNaive(*features, features_traj_, desired_index_, pre_time_fraction);
//                 
                    // Debug
                    float add_x_diff = estimated_pose[0] - odom->pose.pose.position.x;
                    float add_y_diff = estimated_pose[1] - odom->pose.pose.position.y;
                    float add_pose_diff = sqrt(add_x_diff*add_x_diff+add_y_diff*add_y_diff);
                    
                    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    double angle_diff = std::abs(yaw - estimated_pose[2]);
                    
                    std::cout << "Pose difference between curr and estimated: position " << add_pose_diff << ", angle " << angle_diff / M_PI * 180 << std::endl;
                    
                    ros::Time end_fd = ros::Time::now();
                    ros::Duration d = end_fd - start_fd;
                    traj_header_.stamp += d;
                }
                
                break;
                
            case ODOM_POSE or SLAM_POSE:
            {
                ros::WallTime start_fd = ros::WallTime::now();
                featureAddingNaiveGroundTruth(*features, features_traj_, desired_index_, pre_time_fraction, odom);
                ros::WallTime end_fd = ros::WallTime::now();
                ros::WallDuration d = end_fd - start_fd;
                traj_header_.stamp += ros::Duration((double)d.toNSec() / 1000000000);
                
                break;
            }
            case LOCAL_EST_POSE:
                
                break;
                
            default:
            {
                ros::Time start_fd = ros::Time::now();
                featureAddingNaiveGroundTruth(*features, features_traj_, desired_index_, pre_time_fraction, odom);
                ros::Time end_fd = ros::Time::now();
                ros::Duration d = end_fd - start_fd;
                traj_header_.stamp += d;
            }
        }
    }
    
    bool VisualServoController::run_image(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom, double & pre_time_fraction, geometry_msgs::Twist& cmd, bool force_stop)
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
        desired_index_ = curr_feature_traj_stats.curr_feature_traj_idx;
        double pre_time_fraction_tmp = curr_feature_traj_stats.pre_time_fraction;
        desired_features_ = desired_image.features_;
        tf2::Transform camInit2CamDes = desired_image.cam2cam_;
//         vd_ = desired_image.vd_;
        std::cout << "Desired velocity: " << vd_ << std::endl;
        
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
        
//         if(!flags_modes_params_.uncertainty_enabled_)
//         {
//             cmd = computeCmd();
//             retries_counter_ = 0;
//         }
//         else
//         {
//             cv::Mat covMat;
//             calCovMat(filtered_desired_features_, filtered_current_features_, camInit2CamDes, covMat);
//             cmd = computeCmd(covMat);
//             retries_counter_ = 0;
//         }
        
        cmd = computeCmd();
        retries_counter_ = 0;
        return true;
    }
    
    void VisualServoController::run(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom, geometry_msgs::Twist& cmd)
    {
        if(!initialized_ || !local_traj_registered_)
        {
            cmd = stop();
            return;
        }
        
        if(checkFinalGoalReached(odom))
        {
            cmd = stop();
            local_traj_registered_ = false;
            goal_reached_ = true;
            return;
        }
        
        updateFeatures(features);
        std::cout << "Num of current features: " << current_features_.tracked_list.size() << std::endl;
        
        if(checkNextGoalReached(odom))
        {
            std::cout << "Milestone reached. Generate new desired features." << std::endl;
            milestone_reached_ = true;
            
            updateNextGoal();
            generateDesiredFeatures(odom, next_goal_);
//             filterFeatures(desired_features_, current_features_, filtered_desired_features_, filtered_current_features_);
        }
        else
        {
            if(num_filtered_pts_ < 10)
            {
                std::cout << "Num of filtered pts is too low, regenerate desired features." << std::endl;
                generateDesiredFeatures(odom, next_goal_);
            }
            else
            {
                filterFeatures(desired_features_, current_features_, filtered_desired_features_, filtered_current_features_);
            }
        }
        
        assert(filtered_current_features_.tracked_list.size() == filtered_desired_features_.tracked_list.size());
        num_filtered_pts_ = filtered_current_features_.tracked_list.size();
        std::cout << "Num of features used: " << num_filtered_pts_ << std::endl;
        
        if(num_filtered_pts_ == 0)
        {
            cmd = stop();
            std::cout << "No current features can be observed in the desired image." << std::endl;
            return;
        }
        
        cmd = computeCmd();
    }
    
    void VisualServoController::updateFeatures(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features)
    {
        current_features_ = *features;
    }
    
    geometry_msgs::Twist VisualServoController::stop()
    {
        geometry_msgs::Twist zero_cmd;
        zero_cmd.linear.x = 0;
        zero_cmd.linear.y = 0;
        zero_cmd.linear.z = 0;
        zero_cmd.angular.x = 0;
        zero_cmd.angular.y = 0;
        zero_cmd.angular.z = 0;
        
        return zero_cmd;
    }
    
    geometry_msgs::Twist VisualServoController::computeCmd()
    {
        cv::Mat image_jacobian;
        generateImageJacobian(image_jacobian);
        
        double trans[12] = {0, -tx_, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0};
        cv::Mat trans_mat = cv::Mat(6, 2, CV_64FC1, trans);
        
//         std::cout << "image jacobian rows: " << image_jacobian.rows << std::endl;
        assert(image_jacobian.cols == trans_mat.rows);
        
        cv::Mat image_jacobian_trans = image_jacobian * trans_mat;
        cv::Mat image_j_v = image_jacobian_trans.col(0).clone();
        cv::Mat image_j_w = image_jacobian_trans.col(1).clone();
        cv::Mat image_j_w_inv;
        cv::invert(image_j_w, image_j_w_inv, cv::DECOMP_SVD);
        
        cv::Mat curr_feat_mat = featuresToMat(filtered_current_features_);
        cv::Mat desired_feat_mat = featuresToMat(filtered_desired_features_);
        
//         std::cout << "curr_mat: " << curr_feat_mat.rows << ", desired_mat: " << desired_feat_mat.rows << std::endl;
        
        assert(curr_feat_mat.rows == desired_feat_mat.rows);
        
        cv::Mat error = curr_feat_mat - desired_feat_mat;
        
//         std::cout << image_jacobian << std::endl;
        
        return controlLaw(error, image_j_w_inv, image_j_v);
    }
    
    void VisualServoController::generateImageJacobian(cv::Mat& image_jacobian)
    {
        int num_features = num_filtered_pts_;
        image_jacobian = cv::Mat::zeros(2*num_features, 6, CV_64FC1);
        
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
        }
    }
    
    cv::Mat VisualServoController::featuresToMat(const sparse_stereo_msgs::TrackedPointList& features)
    {
        int num_features = features.tracked_list.size();
        cv::Mat out_mat = cv::Mat::zeros(2*num_features, 1, CV_64FC1);
        
        for(size_t i = 0; i < num_features; i++)
        {
            sparse_stereo_msgs::TrackedPoint pt = features.tracked_list[i];
            out_mat.at<double>(i*2, 0) = pt.u_l;
            out_mat.at<double>(i*2+1, 0) = pt.v_l;
        }
        
        return out_mat;
    }
    
    geometry_msgs::Twist VisualServoController::controlLaw(const cv::Mat& error, const cv::Mat& j_w_inv, const cv::Mat& j_v)
    {
        cv::Mat omega_mat = - j_w_inv * ((double) lambda_ * error + (double) vd_ * j_v);
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
