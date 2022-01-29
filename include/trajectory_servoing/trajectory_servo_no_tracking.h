#ifndef _TRAJECTORY_SERVO_NO_TRACKING_
#define _TRAJECTORY_SERVO_NO_TRACKING_

#include <trajectory_servoing/trajectory_servo_feedforward.h>

// #include <tf/Transform.h>

typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
using namespace trajectory_servo_enum;

namespace trajectory_servo_no_tracking_core
{
    using namespace trajectory_servo_core;
    
    class VisualServoNoTrackingController : public VisualServoControllerFeedforward
    {
    public:
        VisualServoNoTrackingController(tf_buffer_ptr tf_buffer);
        ~VisualServoNoTrackingController() {};
        
        
        
    };
    
}


#endif // _TRAJECTORY_SERVO_NO_TRACKING_
