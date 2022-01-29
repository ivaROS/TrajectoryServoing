#include "trajectory_servoing/trajectory_servo_no_tracking_navigator.h"

int main(int argc, char **argv)
{
    std::string name= "trajectory_servo_no_tracking_navigator";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    trajectory_servo_navigator::StereoVSNoTrackingNavigator navigator(nh, pnh);
    navigator.onInit();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
