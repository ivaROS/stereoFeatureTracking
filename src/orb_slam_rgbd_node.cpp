#include "orb_slam_rgbd_processor.h"

int main(int argc, char **argv)
{
    std::string name= "orb_slam_rgbd";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    orb_slam_rgbd::RGBDProc processor(nh, pnh);
    processor.onInit();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

	return 0;
}


