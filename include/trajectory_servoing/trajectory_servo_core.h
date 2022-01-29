#ifndef _TRAJECTORY_SERVO_CORE_
#define _TRAJECTORY_SERVO_CORE_

#include <boost/thread/mutex.hpp>

#include <cmath>
#include <algorithm>

#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sparse_stereo_msgs/TrackedPoint.h>
#include <sparse_stereo_msgs/TrackedPointList.h>
#include <pips_trajectory_msgs/trajectory_points.h>
#include <pips_trajectory_msgs/trajectory_point.h>

#include <image_geometry/stereo_camera_model.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_path_msg/tf2_path_msg.h>

#include <trajectory_servoing/trajectory_servo_enum.h>

// #include <tf/Transform.h>

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
using namespace trajectory_servo_enum;

namespace trajectory_servo_core
{
    struct params
    {
        params(){}
        
        params(int goal_check_mode,
               bool uncertainty_enabled, 
               bool feat_traj_reg_enabled, int feat_traj_reg_mode, int feat_traj_reg_thresh,
               bool feat_traj_reg_adding_enabled = false)
        {
            goal_check_mode_ = static_cast<GoalCheckMode>(goal_check_mode);

            uncertainty_enabled_ = uncertainty_enabled;
            
            feat_traj_reg_enabled_ = feat_traj_reg_enabled;
            feat_traj_reg_mode_ = static_cast<FeatureTrajRegenerationMode>(feat_traj_reg_mode);
            feat_traj_reg_thresh_ = feat_traj_reg_thresh;
            
            feat_traj_reg_adding_enabled_ = feat_traj_reg_adding_enabled;
        }
        
        ~params(){}
        
        GoalCheckMode goal_check_mode_;

        // Considering uncertainty
        bool uncertainty_enabled_;
        
        // Feature trajectory regeneration
        bool feat_traj_reg_enabled_;
        FeatureTrajRegenerationMode feat_traj_reg_mode_;
        int feat_traj_reg_thresh_;
        
        // Feature adding: mix the current features and old feature for regeneration
        bool feat_traj_reg_adding_enabled_;
    };
    
    struct FeaturesTraj
    {
        FeaturesTraj()
        {
            valid_ = false;
        }
        FeaturesTraj(sparse_stereo_msgs::TrackedPointList features, ros::Duration time, float vd, float wd, float x, float y, float theta, tf2::Transform cam2cam)
        {
            features_ = features;
            time_ = time;
            vd_ = vd;
            wd_ = wd;
            x_ = x;
            y_ = y;
            theta_ = theta;
            cam2cam_ = cam2cam;
            valid_ = true;
        }
        ~FeaturesTraj(){}
        
        std::string frame_id_;
        sparse_stereo_msgs::TrackedPointList features_;
        ros::Duration time_;
        float vd_;
        float wd_;
        float x_;
        float y_;
        float theta_;
        tf2::Transform cam2cam_;
        bool valid_;
    };

    struct desiredVelocityCmd
    {
        double vd_, wd_;

        desiredVelocityCmd(double vd, double wd)
        {
            vd_ = vd;
            wd_ = wd;
        }
    };
    
    struct CurrFeatureTrajStats
    {
        int curr_feature_traj_idx;
        double pre_time_fraction;
    };
    
    class VisualServoController
    {
    public:
        VisualServoController(tf_buffer_ptr tf_buffer);
        ~VisualServoController();
        
        virtual void init();
        void reset();
        void initialization(const nav_msgs::Path local_traj);
        void initialization(const pips_trajectory_msgs::trajectory_points local_traj);
        void run(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom, geometry_msgs::Twist& cmd);
        virtual bool run_image(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom, double & pre_time_fraction, geometry_msgs::Twist& cmd, bool force_stop = true);

        int getNumFilteredFeatures() { return num_filtered_pts_; }
        bool checkRegeneration(int num_filtered_pts);
        void featureRegeneration(double pre_time_fraction, const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom);
        
        void registerCameraBaseTrans(const geometry_msgs::TransformStamped& trans);
        void registerStereoCameraModel(const boost::shared_ptr<image_geometry::StereoCameraModel>& stereo);
        void registerLocalTraj(const nav_msgs::Path local_traj);
        void registerLocalTraj(const pips_trajectory_msgs::trajectory_points local_traj);
        void removeLocalTraj();
        void updateFeatures(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features);
        
        void updateNextGoal();
        bool checkNextGoalReached(const nav_msgs::Odometry::ConstPtr& odom);
        bool checkFinalGoalReached(const nav_msgs::Odometry::ConstPtr& odom);
        bool checkFinalGoalReached();
        geometry_msgs::Twist stop();
        
        // double getDesiredFeatures(const std_msgs::Header& current_header, FeaturesTraj& post_features, CurrFeatureTrajStats& curr_feature_traj_stats);
        desiredVelocityCmd getDesiredFeatures(const std_msgs::Header& current_header, FeaturesTraj& post_features, CurrFeatureTrajStats& curr_feature_traj_stats);
        
        void generateAllFeatures(const sparse_stereo_msgs::TrackedPointList::ConstPtr& current_features, 
                                 const pips_trajectory_msgs::trajectory_points& traj, const nav_msgs::Odometry::ConstPtr& odom);
        
        void generateDesiredFeatures(const sparse_stereo_msgs::TrackedPointList::ConstPtr& current_features, 
                                     const pips_trajectory_msgs::trajectory_point& traj_pose,
                                     FeaturesTraj& feature_traj, tf2::Transform curr_trans);
        void generateDesiredFeatures(const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::PoseStamped& goal_pose);
        void filterFeatures(const sparse_stereo_msgs::TrackedPointList& f1, const sparse_stereo_msgs::TrackedPointList& f2,
                            sparse_stereo_msgs::TrackedPointList& f1_out, sparse_stereo_msgs::TrackedPointList& f2_out);
        
        static double distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
        static double distance(const sparse_stereo_msgs::TrackedPoint& p1, sparse_stereo_msgs::TrackedPoint& p2);
        static double distance(const geometry_msgs::Pose& p1, pips_trajectory_msgs::trajectory_point& p2);
        
        // feature regeneration
        std::vector<double> featureAddingNaive(const sparse_stereo_msgs::TrackedPointList& current_features, std::vector<FeaturesTraj>& features_traj, int curr_feature_traj_idx, double pre_time_fraction);
        void featureAddingNaiveGroundTruth(const sparse_stereo_msgs::TrackedPointList& current_features, std::vector<FeaturesTraj>& features_traj, int curr_feature_traj_idx, double pre_time_fraction, const nav_msgs::Odometry::ConstPtr& odom);
        
        // uncertainty
//         void calCovMat(sparse_stereo_msgs::TrackedPointList& filtered_desired_features, sparse_stereo_msgs::TrackedPointList& filtered_current_features, 
//                        tf2::Transform camInit2CamDes, cv::Mat& covMat);
//         geometry_msgs::Twist computeCmd(cv::Mat& covMat);
        
        geometry_msgs::Twist computeCmd();
        void generateImageJacobian(cv::Mat& image_jacobian);
        cv::Mat featuresToMat(const sparse_stereo_msgs::TrackedPointList& features);
        geometry_msgs::Twist controlLaw(const cv::Mat& error, const cv::Mat& j_w_inv, const cv::Mat& j_v);
//         geometry_msgs::Twist controlLaw(const cv::Mat& error, const cv::Mat& j_w_inv, const cv::Mat& j_w, const cv::Mat& j_v, const cv::Mat& Omega);
        
        // Debug
        void getAllFeatures(sparse_stereo_msgs::TrackedPointList& curr_features, sparse_stereo_msgs::TrackedPointList& desired_features, 
                            sparse_stereo_msgs::TrackedPointList& filtered_current_features, sparse_stereo_msgs::TrackedPointList& filtered_desired_features)
        {
            curr_features = current_features_;
            desired_features = desired_features_;
            filtered_current_features = filtered_current_features_;
            filtered_desired_features = filtered_desired_features_;
        }
        
        void getNextGoal(geometry_msgs::PoseStamped& next_goal)
        {
            next_goal = next_goal_;
        }
        
        bool initialized_ = false;
        bool local_traj_registered_ = false;
        bool goal_reached_ = false;
        bool milestone_reached_ = false;
        tf_buffer_ptr tfBuffer_;
        
        int retries_counter_;
        
        // Flags and params for different modes
        void setFlagsAndModes(params flags_modes_params)
        {
            flags_modes_params_ = flags_modes_params;
        }
        void setNoiseStd(double noise_std)
        {
            noise_std_=noise_std;
        }
        
        virtual void setUncertaintyParams(double sigmaMu, double sigmaNu, double sigmaD) {}
        
        params flags_modes_params_;
        double noise_std_;

        
    public:
        bool goalReached() { return goal_reached_; }
        
        void setConstLinearVel(float vd) { vd_ = vd; }
        void setLambda(float lambda) { lambda_ = lambda; }
        void setMilestoneThresh(double milestone_thresh) { milestone_thresh_ = milestone_thresh; }
        void setFinalGoalThresh(double final_goal_thresh) { final_goal_thresh_ = final_goal_thresh; }
        void setTx(float tx) { tx_ = tx; }
        
        float getConstLinearVel() { return vd_; }
        virtual double getDesiredAngularVel() {};
        float getLambda() { return lambda_; }
        double getMilestoneThresh() { return milestone_thresh_; }
        double getFinalGoalThresh() { return milestone_thresh_; }
        float getTx() { return tx_; }

        int getDesiredIdx() {return desired_index_;}
        int getCurrIdx() {return curr_index_;}
        
    protected:
        boost::mutex core_mutex_;
        
        tf2::Transform camera_base_trans_;
        boost::shared_ptr<image_geometry::StereoCameraModel> stereo_camera_model_;
        nav_msgs::Path local_traj_;
        pips_trajectory_msgs::trajectory_points local_ni_traj_;
        sparse_stereo_msgs::TrackedPointList current_features_, filtered_current_features_;
        sparse_stereo_msgs::TrackedPointList desired_features_, filtered_desired_features_;
        
        int total_num_poses_, current_pose_idx_, num_filtered_pts_;
        geometry_msgs::PoseStamped next_goal_;
        
        std_msgs::Header traj_header_;
        std::vector<FeaturesTraj> features_traj_;
        int curr_index_, desired_index_;
        
        // camera param
        double fx_, fy_;
        double cx_, cy_;
        int image_height_, image_width_;
        cv::Mat cameraMat_, cameraMatInv_;
        double baseline_;
        
        // Params
        float vd_, lambda_, tx_, ty_;
        double final_goal_thresh_, milestone_thresh_;
        
        // Debug
        float dist;
        int num_pose_diff_compute;
        
    };
    
}


#endif // _TRAJECTORY_SERVO_CORE_
