#ifndef _ORB_SLAM_RGBD_PROCESSOR_
#define _ORB_SLAM_RGBD_PROCESSOR_

#include "dynamic_reconfigurable_param.h"

#include "System.h"
#include "MapPublisher.h"

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
    #include <boost/thread/lock_guard.hpp>
#endif

#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sparse_stereo_msgs/TrackedPoint.h>
#include <sparse_stereo_msgs/TrackedPointList.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace orb_slam_rgbd {

constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
    
using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class RGBDProc
{
public:
    RGBDProc(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~RGBDProc()
    {
        if(false)
        {
            SLAM->SaveKeyFrameTrajectoryTUM( traj_file_ + "_KeyFrameTrajectory.txt" );
            SLAM->SaveTrackingLog( traj_file_ + "_Log.txt" );
        }
        
        SLAM->Shutdown();
    };
    bool onInit();
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int frame_counter_;
    int restart_frame_counter_;
    bool visualize_enable_;
    bool optimize_pose_enable_;
    
    std::string rgb_image_topic_;
    std::string depth_image_topic_;
    std::string depth_cam_info_topic_;
//     std::string right_cam_info_topic_;
    std::string local_frame_;
    
    boost::shared_ptr<image_geometry::PinholeCameraModel> pinhole_camera_model_;

private:
    std::string name_= "rgbd";
    ros::NodeHandle nh_, pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_, it_vis_, it_vis_tracked_;

    // Subscriptions
    image_transport::SubscriberFilter sub_rgb_image_, sub_depth_image_;
    message_filters::Subscriber<CameraInfo> sub_depth_info_;
    typedef ExactTime<Image, Image, CameraInfo> ExactPolicy;
    typedef ApproximateTime<Image, Image, CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    
    // Publications
    boost::mutex connect_mutex_;
    ros::Publisher pub_points2_;
    ros::Publisher pub_sparse_lines_;
    image_transport::CameraPublisher pub_depth_;

    // Visualization
    image_transport::Publisher pub_vis_matching_result_, pub_vis_tracked_result_;
    cv::Mat matched_img_, tracked_img_;

    // Dynamic reconfigure
    boost::recursive_mutex config_mutex_;
    typedef sparse_stereo::SparseStereoTesterConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    
    // ORB_SLAM2
    ORB_SLAM2::System* SLAM;
    double timeStamp;
    std::string traj_file_;
    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
    ros::Publisher mpFrameWithInfoPublisher;
    
    cv::Mat sparse_depth_;
    
    ros::Publisher pub_tracked_points_;
    
    boost::shared_ptr<PointCloud> points_msg_;
    boost::shared_ptr<visualization_msgs::Marker> lines_msg_;
    
//     double maxDisparity_prop;
//     bool mask_enabled_ = true;
//     bool crosscheck_enabled_ = true;
//     bool both_ = false;
//     bool parallelism_enabled_ = true;

private:

    void connectCb();

    void imageCb(const ImageConstPtr& rgb_image_msg, const ImageConstPtr& depth_image_msg, const CameraInfoConstPtr& depth_info_msg);

    inline bool isValidPoint(const float z)
    {
        // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
        // and zero disparities (point mapped to infinity).
        return z != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(z);
    }
    void configCb(Config &config, uint32_t level);
};
}

#endif //_ORB_SLAM_RGBD_PROCESSOR_


