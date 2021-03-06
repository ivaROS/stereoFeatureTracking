#include "local_octomap_generator.h"

int main(int argc, char **argv)
{
    std::string name= "local_octomap_generator";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pips_sparse_stereo::local_octomap::OctomapGenerator generator(nh, pnh);
    generator.init();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();


	return 0;
}
