#ifndef TRAJ_SERVO_ENUM_H
#define TRAJ_SERVO_ENUM_H

namespace trajectory_servo_enum{
    
    enum FeatureTrajRegenerationMode {
        NAIVE_PERFECT_POSE,
        ODOM_POSE,
        SLAM_POSE,
        LOCAL_EST_POSE
    };
    
    enum FeatureAddingMode
    {
    };

    enum GoalCheckMode
    {
        USE_ODOM,
        LAST_POSE
    };
}

#endif
