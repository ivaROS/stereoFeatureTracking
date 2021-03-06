#include "pips_sparse_stereo_processor.h"

int main(int argc, char **argv)
{
    std::string name= "pips_sparse_stereo";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pips_sparse_stereo::SparseStereoProc processor(nh, pnh);
    processor.onInit();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
