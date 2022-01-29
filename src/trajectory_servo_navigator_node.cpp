#include "trajectory_servoing/trajectory_servo_navigator.h"

int main(int argc, char **argv)
{
    std::string name= "trajectory_servo_navigator";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    trajectory_servo_navigator::StereoVSNavigator navigator(nh, pnh);
    navigator.onInit();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
