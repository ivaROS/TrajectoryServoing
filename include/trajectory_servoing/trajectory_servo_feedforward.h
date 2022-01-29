#ifndef _TRAJECTORY_SERVO_FEEDFORWARD_
#define _TRAJECTORY_SERVO_FEEDFORWARD_

#include <trajectory_servoing/trajectory_servo_core.h>

// #include <tf/Transform.h>

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
using namespace trajectory_servo_enum;

namespace trajectory_servo_core
{
//     struct params_uncert : public params
//     {
//         params_uncert() : params(){}
//         ~params_uncert(){}
//         
//         params_uncert(bool uncertainty_enabled, double sigmaMu, double sigmaNu, double sigmaD,
//                bool feat_traj_reg_enabled, int feat_traj_reg_mode, int feat_traj_reg_thresh,
//                bool feat_traj_reg_adding_enabled = false) : 
//                params(uncertainty_enabled,
//                       feat_traj_reg_enabled, feat_traj_reg_mode, feat_traj_reg_thresh,
//                       feat_traj_reg_adding_enabled)
//         {
//             sigmaMu_ = sigmaMu;
//             sigmaNu_ = sigmaNu;
//             sigmaD_ = sigmaD;
//             
// //             params tmp(uncertainty_enabled, 
// //                       feat_traj_reg_enabled, feat_traj_reg_mode, feat_traj_reg_thresh,
// //                       feat_traj_reg_adding_enabled);
// //             
// //             params_ = tmp;
//         }
//         
// //         params getParams()
// //         {
// //             return params_;
// //         }
//         
//         double sigmaMu_, sigmaNu_, sigmaD_;
// //         params params_;
//     };
    
    class VisualServoControllerFeedforward : public VisualServoController
    {
    public:
        VisualServoControllerFeedforward(tf_buffer_ptr tf_buffer);
        ~VisualServoControllerFeedforward(){};
        
        virtual void init();
        
        virtual bool run_image(const sparse_stereo_msgs::TrackedPointList::ConstPtr& features, const nav_msgs::Odometry::ConstPtr& odom, double & pre_time_fraction, geometry_msgs::Twist& cmd, bool force_stop = true);
        
        virtual double getDesiredAngularVel(){ return wd_; }
        // feedforward
        geometry_msgs::Twist computeCmd(double wd);
        void generateImageJacobian(cv::Mat& image_jacobian, cv::Mat& desired_image_jacobian);
        
        geometry_msgs::Twist controlLaw(const cv::Mat& error, const cv::Mat& j_w_inv, const cv::Mat& j_v, const cv::Mat& d_j_v, const cv::Mat& d_j_w);

    protected:
        double wd_;
        
    };
    
}


#endif // _TRAJECTORY_SERVO_UNCERTAINTY_

