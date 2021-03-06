#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class ImageBlur
{
private:
    std::string name_ = "image_blur";
    ros::NodeHandle nh_, pnh_;
  
    message_filters::Subscriber<Image> sub_l_image_, sub_r_image_;
    message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
    typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
    typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
  
    image_transport::ImageTransport it_;  
    
    std::string left_out = name_ + "/left/image_raw";
    std::string left_cam_out = name_ + "/left/camera_info";
    std::string right_out = name_ + "/right/image_raw";
    std::string right_cam_out = name_ + "/right/camera_info";
    
    image_transport::Publisher pub_l_, pub_r_;
    ros::Publisher pub_cam_l_, pub_cam_r_; 
    
    cv::Mat l_image_, r_image_;
    sensor_msgs::ImagePtr left_img_msg_, right_img_msg_;
    sensor_msgs::CameraInfoPtr left_info_msg_, right_info_msg_;

public:
    ImageBlur(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh),
    it_(nh)
    { }

    ~ImageBlur() { }
    
    bool init()
    {
        sub_l_image_.subscribe(nh_, "/multisense_sl/camera/left/image_raw", 1);
        sub_l_info_.subscribe(nh_, "/multisense_sl/camera/left/camera_info", 1);
        sub_r_image_.subscribe(nh_, "/multisense_sl/camera/right/image_raw", 1);
        sub_r_info_.subscribe(nh_, "/multisense_sl/camera/right/camera_info", 1);
        
        // Synchronize inputs. Topic subscriptions happen on demand in the connection
        // callback. Optionally do approximate synchronization.
        int queue_size;
        pnh_.param("queue_size", queue_size, 12);
        bool approx;
        pnh_.param("approximate_sync", approx, true);
        if (approx)
        {
            approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                        sub_l_image_, sub_l_info_,
                                                        sub_r_image_, sub_r_info_) );
            approximate_sync_->registerCallback(boost::bind(&ImageBlur::imageCb,
                                                            this, _1, _2, _3, _4));
        }
        else
        {
            exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                            sub_l_image_, sub_l_info_,
                                            sub_r_image_, sub_r_info_) );
            exact_sync_->registerCallback(boost::bind(&ImageBlur::imageCb,
                                                    this, _1, _2, _3, _4));
        }
        
        pub_l_ = it_.advertise(left_out, 1);
        pub_r_ = it_.advertise(right_out, 1);
        pub_cam_l_ = nh_.advertise<sensor_msgs::CameraInfo>(left_cam_out, 1);
        pub_cam_r_ = nh_.advertise<sensor_msgs::CameraInfo>(right_cam_out, 1);

        return 1;
    }

private:
    void imageCb(const ImageConstPtr& l_image_msg,
                 const CameraInfoConstPtr& l_info_msg,
                 const ImageConstPtr& r_image_msg,
                 const CameraInfoConstPtr& r_info_msg)
    {
        if(l_image_msg->encoding == "mono8")
        {
            l_image_ = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
            r_image_ = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;
        }
        else if(l_image_msg->encoding == "bgr8")
        {
            l_image_ = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8)->image;
            r_image_ = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        
        cv::Size ksize(5, 5);
        double sigmaX = 2;
        cv::GaussianBlur(l_image_, l_image_, ksize, sigmaX);
        cv::GaussianBlur(r_image_, r_image_, ksize, sigmaX);
        
        left_info_msg_ = boost::make_shared<sensor_msgs::CameraInfo>(*l_info_msg);;
        right_info_msg_ = boost::make_shared<sensor_msgs::CameraInfo>(*r_info_msg);;
        
        left_img_msg_ = cv_bridge::CvImage(l_image_msg->header, l_image_msg->encoding, l_image_).toImageMsg();
        right_img_msg_ = cv_bridge::CvImage(r_image_msg->header, r_image_msg->encoding, r_image_).toImageMsg();
        
        ROS_INFO_STREAM_NAMED(name_, "Image blured.");
        
        pub_l_.publish(left_img_msg_);
        pub_cam_l_.publish(left_info_msg_);
        pub_r_.publish(right_img_msg_);
        pub_cam_r_.publish(right_info_msg_);

    }
};

int main(int argc, char** argv)
{
    std::string name= "image_blur";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ImageBlur processor(nh, pnh);
    processor.init();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

	return 0;
}

