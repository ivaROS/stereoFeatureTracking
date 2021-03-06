#ifndef _PIPS_SPARSE_STEREO_PROCESSOR_
#define _PIPS_SPARSE_STEREO_PROCESSOR_

#include "dynamic_reconfigurable_param.h"

#include "pinholeStereoCamera.h"
#include "stereoFrameHandler.h"
#include "stereoFrame.h"
#include "timer.h"

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
    #include <boost/thread/lock_guard.hpp>
#endif

#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <sparse_stereo/SparseStereoTesterConfig.h>
// #include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

#include <sparse_stereo_msgs/TrackedPoint.h>
#include <sparse_stereo_msgs/TrackedPointList.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace pips_sparse_stereo {

constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
    
using namespace stvo;
using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class SparseStereoProc
{
public:
    SparseStereoProc(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~SparseStereoProc(){};
    bool onInit();
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int frame_counter_;
    int restart_frame_counter_;
    bool visualize_enable_;
    bool optimize_pose_enable_;
    
    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string left_cam_info_topic_;
    std::string right_cam_info_topic_;
    std::string local_frame_;
    

private:
    std::string name_= "sparse_stereo";
    ros::NodeHandle nh_, pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_, it_vis_, it_vis_tracked_;

    // Subscriptions
    image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
    message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
    typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
    typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    
    // Publications
    boost::mutex connect_mutex_;
    ros::Publisher pub_points2_;
    ros::Publisher pub_sparse_lines_;
    image_transport::CameraPublisher pub_depth_;

    ros::Publisher tracked_features_pub_;

    // Visualization
    image_transport::Publisher pub_vis_matching_result_, pub_vis_tracked_result_;
    cv::Mat matched_img_, tracked_img_;

    // Dynamic reconfigure
    boost::recursive_mutex config_mutex_;
    typedef sparse_stereo::SparseStereoTesterConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    // stvo
    PinholeStereoCamera* cam_;
    StereoFrameHandler* StVO_;
    
    // Processing state
    Timer timer_;
    
    cv::Mat l_image_;
    cv::Mat r_image_;
    
    std::vector< PointFeature* > stereo_pt_;
    std::vector< LineFeature* > stereo_ls_;
    
    cv::Mat sparse_depth_;
    
    boost::shared_ptr<PointCloud> points_msg_;
    boost::shared_ptr<visualization_msgs::Marker> lines_msg_;
    
    matching_params_ptr params_;
//     double maxDisparity_prop;
//     bool mask_enabled_ = true;
//     bool crosscheck_enabled_ = true;
//     bool both_ = false;
//     bool parallelism_enabled_ = true;

private:

    void connectCb();

    void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

    inline bool isValidPoint(const float z)
    {
        // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
        // and zero disparities (point mapped to infinity).
        return z != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(z);
    }
    void configCb(Config &config, uint32_t level);
    
    void generate3DPtLi(PinholeStereoCamera* cam, std::vector<PointFeature*>& stereo_pt, std::vector<LineFeature*>& stereo_ls, boost::shared_ptr<PointCloud>& points_msg, boost::shared_ptr<visualization_msgs::Marker>& lines_msg, cv::Mat& sparse_depth);
};
}

#endif //_PIPS_SPARSE_STEREO_PROCESSOR_
