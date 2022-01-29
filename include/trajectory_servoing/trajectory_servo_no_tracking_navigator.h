#ifndef TRAJECTORY_SERVO_NO_TRACKING_NAVIGATOR_H
#define TRAJECTORY_SERVO_NO_TRACKING_NAVIGATOR_H

#include "pinholeStereoCamera.h"
#include "stereoFrame.h"
#include "matching.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "trajectory_servoing/trajectory_servo_navigator.h"
#include "trajectory_servoing/trajectory_servo_no_tracking.h"
#include "trajectory_servoing/cubic_spline_interpolator.h"

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
typedef std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

using namespace stvo;

namespace trajectory_servo_navigator
{
    class StereoVSNoTrackingNavigator : public StereoVSNavigator
    {
    
    public:
        StereoVSNoTrackingNavigator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        ~StereoVSNoTrackingNavigator(){};
        virtual bool onInit();
        
    public:
        
        void imageCB(const sensor_msgs::Image::ConstPtr& l_img, const sensor_msgs::Image::ConstPtr& r_img,
                     const sensor_msgs::CameraInfo::ConstPtr& l_info, const sensor_msgs::CameraInfo::ConstPtr& r_info);
        
        void matchF2FPoints();
        sparse_stereo_msgs::TrackedPointList::ConstPtr generateFeatureMsg(const std::vector< PointFeature* >& stereo_pt, std_msgs::Header header);
        
    protected:
        std::string name_= "stereo_visual_servo_no_tracking_navigator";
        
        message_filters::Subscriber<Image> sub_l_img_, sub_r_img_;
//         typedef ExactTime<sparse_stereo_msgs::TrackedPointList, CameraInfo> ExactPolicy;
        typedef ApproximateTime<Image, Image, CameraInfo, CameraInfo> ApproximatePolicyImage;
//         typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
        typedef message_filters::Synchronizer<ApproximatePolicyImage> ApproximateSyncImage;
//         boost::shared_ptr<ExactSync> exact_sync_;
        boost::shared_ptr<ApproximateSyncImage> approximate_sync_img_;

        ros::Publisher raw_matching_img_pub_;
        
        // Feature matching
        int frame_counter_;
        StereoFrame* curr_frame_;
        StereoFrame* first_frame_;
        PinholeStereoCamera *cam_;

        std::vector< PointFeature* > stereo_pt_;
        std::vector< PointFeature* > first_stereo_pt_;
        std::vector< PointFeature* > matched_pt_;
    };
    
}

#endif // TRAJECTORY_SERVO_NO_TRACKING_NAVIGATOR_H
