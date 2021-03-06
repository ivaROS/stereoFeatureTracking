#include "orb_slam_sparse_stereo_processor.h"


namespace orb_slam_sparse_stereo {

    SparseStereoProc::SparseStereoProc(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh)
    {
        frame_counter_ = 0;
        restart_frame_counter_ = 0;
//         visualize_enable_ = true;
        
        stereo_camera_model_ = boost::make_shared<image_geometry::StereoCameraModel>();
        
        points_msg_ = boost::make_shared<PointCloud>();
        lines_msg_ = boost::make_shared<visualization_msgs::Marker>();
    }
    
    bool SparseStereoProc::onInit()
    {
        it_.reset(new image_transport::ImageTransport(nh_));
        it_vis_.reset(new image_transport::ImageTransport(nh_));
        it_vis_tracked_.reset(new image_transport::ImageTransport(nh_));
    
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
            approximate_sync_->registerCallback(boost::bind(&SparseStereoProc::imageCb,
                                                            this, _1, _2, _3, _4));
        }
        else
        {
            exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                            sub_l_image_, sub_l_info_,
                                            sub_r_image_, sub_r_info_) );
            exact_sync_->registerCallback(boost::bind(&SparseStereoProc::imageCb,
                                                    this, _1, _2, _3, _4));
        }
        
        left_image_topic_ = "/multisense_sl/camera/left/image_raw";
        right_image_topic_ = "/multisense_sl/camera/right/image_raw";
        left_cam_info_topic_ = "/multisense_sl/camera/left/camera_info";
        right_cam_info_topic_ = "/multisense_sl/camera/right/camera_info";
        local_frame_ = "stereo_camera_optical_frame";
            
        pnh_.getParam("left_image_topic", left_image_topic_ );
        pnh_.getParam("right_image_topic", right_image_topic_ );
        pnh_.getParam("left_cam_info_topic", left_cam_info_topic_ );
        pnh_.getParam("right_cam_info_topic", right_cam_info_topic_ );
        pnh_.getParam("local_frame", local_frame_);
        
        pnh_.setParam("left_image_topic", left_image_topic_ );
        pnh_.setParam("right_image_topic", right_image_topic_ );
        pnh_.setParam("left_cam_info_topic", left_cam_info_topic_ );
        pnh_.setParam("right_cam_info_topic", right_cam_info_topic_ );
        pnh_.setParam("local_frame", local_frame_);
        
        std::string vocab_file;
        pnh_.getParam("vocab_file", vocab_file);
        pnh_.setParam("vocab_file", vocab_file);
        
        std::string config_file;
        pnh_.getParam("config_file", config_file);
        pnh_.setParam("config_file", config_file);
        
        int num_f;
        pnh_.getParam("num_features", num_f);
        pnh_.setParam("num_features", num_f);
        
        bool do_rectify;
        pnh_.getParam("rectify", do_rectify);
        pnh_.setParam("rectify", do_rectify);
        
        bool do_viz;
        pnh_.getParam("visualization", do_viz);
        pnh_.setParam("visualization", do_viz);
        
        pnh_.getParam("traj_file", traj_file_);
        pnh_.setParam("traj_file", traj_file_);
        
        std::string map_file;
        pnh_.getParam("map_file", map_file);
        pnh_.setParam("map_file", map_file);
        
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        SLAM = new ORB_SLAM2::System(vocab_file, config_file, ORB_SLAM2::System::STEREO, do_viz);
        SLAM->SetConstrPerFrame(num_f);
        
        // Set up dynamic reconfiguration
        ReconfigureServer::CallbackType f = boost::bind(&SparseStereoProc::configCb,
                                                        this, _1, _2);
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
        reconfigure_server_->setCallback(f);

        // Monitor whether anyone is subscribed to the output
        image_transport::SubscriberStatusCallback connect_cb = boost::bind(&SparseStereoProc::connectCb, this);
        ros::SubscriberStatusCallback connect_cb_info = boost::bind(&SparseStereoProc::connectCb, this);
        // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        
        mpCameraPosePublisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(name_ + "/camera_pose", 100, connect_cb_info, connect_cb_info);
        mpCameraPoseInIMUPublisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(name_ + "/camera_pose_in_imu", 100, connect_cb_info, connect_cb_info);
        mpFrameWithInfoPublisher = nh_.advertise<sensor_msgs::Image>(name_ + "/frame_with_info", 100, connect_cb_info, connect_cb_info);
        
        pub_tracked_points_ = nh_.advertise<sparse_stereo_msgs::TrackedPointList>(name_ + "/tracked_points", 1, connect_cb_info, connect_cb_info);
        pub_points2_ = nh_.advertise<PointCloud>(name_ + "/sparse_points2", 1, connect_cb_info, connect_cb_info);
//         pub_sparse_lines_ = nh_.advertise<visualization_msgs::Marker>(name_ + "/sparse_lines", 1, connect_cb_info, connect_cb_info);
//         pub_depth_ = it_->advertiseCamera(name_ + "/sparse_depth", 1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
//         pub_vis_matching_result_ = it_vis_->advertise(name_ + "/matching_result", 1, connect_cb, connect_cb);
//         pub_vis_tracked_result_ = it_vis_tracked_->advertise(name_ + "/tracked_result", 1, connect_cb, connect_cb);

        return 1;
    }




    // Handles (un)subscribing when clients (un)subscribe
    void SparseStereoProc::connectCb()
    {
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
//         if (pub_points2_.getNumSubscribers() == 0 && pub_depth_.getNumSubscribers() == 0
//             && pub_sparse_lines_.getNumSubscribers() == 0 && pub_vis_matching_result_.getNumSubscribers() == 0)
//         {
//             sub_l_image_.unsubscribe();
//             sub_l_info_ .unsubscribe();
//             sub_r_image_.unsubscribe();
//             sub_r_info_ .unsubscribe();
//         }
//         else if (!sub_l_image_.getSubscriber())
//         if(!sub_l_image_.getSubscriber())
//         {
            // ros::NodeHandle &nh = getNodeHandle();
            // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
            /// @todo Allow remapping left, right?

            // subscribe to image
            ROS_INFO_STREAM_NAMED(name_, "Subscribe to images and camera info.");
            image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
            sub_l_image_.subscribe(*it_, left_image_topic_, 1, hints);
            sub_l_info_ .subscribe(nh_, left_cam_info_topic_, 1);
            sub_r_image_.subscribe(*it_, right_image_topic_, 1, hints);
            sub_r_info_ .subscribe(nh_, right_cam_info_topic_, 1);
//         }
    }



    void SparseStereoProc::imageCb(const ImageConstPtr& l_image_msg,
                                const CameraInfoConstPtr& l_info_msg,
                                const ImageConstPtr& r_image_msg,
                                const CameraInfoConstPtr& r_info_msg)
    {
        
//         if(l_image_msg->encoding == "mono8")
//         {
//             l_image_ = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
//             r_image_ = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;
//         }
//         else if(l_image_msg->encoding == "bgr8")
//         {
//             l_image_ = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8)->image;
//             r_image_ = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::BGR8)->image;
//         }
        
        double latency_trans = ros::Time::now().toSec() - l_image_msg->header.stamp.toSec();
        
        stereo_camera_model_->fromCameraInfo(l_info_msg, r_info_msg);
        
        l_image_ = cv_bridge::toCvShare(l_image_msg)->image;
        r_image_ = cv_bridge::toCvShare(r_image_msg)->image;
        
        cv::Mat l_gray, r_gray;
        if(l_image_msg->encoding == "bgr8")
        {
            cv::cvtColor(l_image_, l_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(r_image_, r_gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            l_gray = l_image_.clone();
            r_gray = r_image_.clone();
        }
        ROS_INFO("Image received and converted.");
        
        cv::Mat pose;
        pose = SLAM->TrackStereo(l_gray,r_gray,l_image_msg->header.stamp.toSec());
        
        if (pose.empty())
        return;

        double latency_total = ros::Time::now().toSec() - l_image_msg->header.stamp.toSec();
        // ROS_INFO("ORB-SLAM Tracking Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());
        // ROS_INFO("Image Transmision Latency: %.03f sec; Total Tracking Latency: %.03f sec", latency_trans, latency_total);
        ROS_INFO("Pose Tracking Latency: %.03f sec", latency_total - latency_trans);
        
        // Obtain the tracked points
        std::vector<cv::KeyPoint> stereo_pt_l;
        std::vector<float> stereo_pt_depth;
        std::vector<long unsigned int> stereo_pt_id;
        SLAM->mpFrameDrawer->GrabTrackedPoints(stereo_pt_l, stereo_pt_depth, stereo_pt_id);
//         stereo_pt_l = SLAM->GetTrackedKeyPointsUn();
        
        sparse_stereo_msgs::TrackedPointList tracked_points;
        tracked_points.header = l_image_msg->header;
        for(size_t i = 0; i < stereo_pt_id.size(); i++)
        {
            bool repeated = false;
            for(size_t j = 0; j < stereo_pt_id.size(); j++)
            {
                if(i != j && stereo_pt_id[i] == stereo_pt_id[j])
                {
//                     ROS_INFO_STREAM("Repeated: " << stereo_pt_id[i] << ", iu: " << stereo_pt_l[i].pt.x << ", iv: " << stereo_pt_l[i].pt.y << ", ju: " << stereo_pt_l[j].pt.x << ", jv: " << stereo_pt_l[j].pt.y);
                    repeated = true;
                    
                }
            }
            if(!repeated)
            {
                sparse_stereo_msgs::TrackedPoint tracked_pt;
                tracked_pt.header = l_image_msg->header;
                tracked_pt.u_l = stereo_pt_l[i].pt.x;
                tracked_pt.v_l = stereo_pt_l[i].pt.y;
                tracked_pt.depth = stereo_pt_depth[i];
                tracked_pt.id = stereo_pt_id[i];
                
                tracked_points.tracked_list.push_back(tracked_pt);
            }
            
        }
        pub_tracked_points_.publish(tracked_points);
        
        ROS_INFO_STREAM("Tracked [ " << tracked_points.tracked_list.size() <<" ] points.");
        
        // Publish camera pose
        cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);
        tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                        Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                        Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
        tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

        tf::Transform tfTcw(M,V);
        geometry_msgs::Transform gmTwc;
        tf::transformTFToMsg(tfTcw, gmTwc);

        geometry_msgs::Pose camera_pose;
        camera_pose.position.x = gmTwc.translation.x;
        camera_pose.position.y = gmTwc.translation.y;
        camera_pose.position.z = gmTwc.translation.z;
        camera_pose.orientation = gmTwc.rotation;
        
        geometry_msgs::PoseWithCovarianceStamped camera_odom;
        camera_odom.header.frame_id = "odom";
        camera_odom.header.stamp = l_image_msg->header.stamp;
        camera_odom.pose.pose = camera_pose;
        
        mpCameraPosePublisher.publish(camera_odom);
        
        // Publish camera_odom_in_imu
        tf::Matrix3x3 Ric(   0, 0, 1,
                -1, 0, 0,
                0, -1, 0);
        tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ), Ric * tfTcw.getOrigin() );
        
        geometry_msgs::Transform gmTwi;
        tf::transformTFToMsg(tfTiw, gmTwi);

        geometry_msgs::Pose camera_pose_in_imu;
        camera_pose_in_imu.position.x = gmTwi.translation.x;
        camera_pose_in_imu.position.y = gmTwi.translation.y;
        camera_pose_in_imu.position.z = gmTwi.translation.z;
        camera_pose_in_imu.orientation = gmTwi.rotation;
        
        geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
        camera_odom_in_imu.header.frame_id = "map";
        camera_odom_in_imu.header.stamp = l_image_msg->header.stamp;
        camera_odom_in_imu.pose.pose = camera_pose_in_imu;
        
        mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);
        
        // Publish image frame with info
        if(visualize_enable_)
        {
            if(mpFrameWithInfoPublisher.getNumSubscribers() != 0)
            {
                if (SLAM != NULL && SLAM->mpFrameDrawer != NULL) {
                    cv::Mat fr_info_cv = SLAM->mpFrameDrawer->DrawFrame();
                    cv_bridge::CvImage out_msg;
                    out_msg.header   = l_image_msg->header; // Same timestamp and tf frame as input image
                    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
                    out_msg.image    = fr_info_cv; // Your cv::Mat
                    mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
                }
            }
            
            if(pub_points2_.getNumSubscribers() != 0)
            {
                points_msg_->header.frame_id = local_frame_;
                points_msg_->height = 1;
                points_msg_->points.clear();
                pcl_conversions::toPCL(l_image_msg->header.stamp, points_msg_->header.stamp);
                
                for(size_t i = 0; i < stereo_pt_l.size(); i++)
                {
                    double disparity = stereo_camera_model_->getDisparity(stereo_pt_depth[i]);
                    cv::Point3d pt;
                    stereo_camera_model_->projectDisparityTo3d(stereo_pt_l[i].pt, disparity, pt);
                    pcl::PointXYZ point(pt.x, pt.y, pt.z);
                    points_msg_->points.push_back(point);
                }
                points_msg_->width = points_msg_->points.size();
                
                pub_points2_.publish(points_msg_);
            }
        }

        

    }
    
    

    void SparseStereoProc::configCb(Config &config, uint32_t level)
    {
//         ROS_INFO_STREAM_NAMED(name_, "Reconfigure Request: ");
        
        visualize_enable_ = config.visualize_enabled;
        optimize_pose_enable_ = config.optimize_pose_enabled;
        
//         params_->detector_method = config.Detector;
//         params_->descriptor_method = config.Descriptor;
//         params_->orb_num = config.orb_num;
// 
// //         mask_enabled_ = config.enableMask;
// //         crosscheck_enabled_ = config.crosscheck;
// //         both_ = config.Both;
// //         parallelism_enabled_ = config.parallelism;
// 
//         params_->range = config.robotRange;
// 
//         params_->epiRange = config.epi_range;
// //         maxDisparity_prop = config.max_disparity_prop;
//         params_->maxDistance = config.max_distance;
//         params_->knn_enabled = config.knn_enabled;
//         params_->k = config.k;
//         params_->repeated_remove = config.repeated_remove;
//         params_->useBFMatcher = config.useBFMatcher;
    }

}


