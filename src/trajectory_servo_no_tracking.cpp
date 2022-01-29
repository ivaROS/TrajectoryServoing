#include <trajectory_servoing/trajectory_servo_no_tracking.h>

namespace trajectory_servo_no_tracking_core
{
    VisualServoNoTrackingController::VisualServoNoTrackingController(tf_buffer_ptr tf_buffer) : 
        VisualServoControllerFeedforward(tf_buffer)
    {
        
    }

}
