#include "pips_sparse_stereo_processor.h"


namespace pips_sparse_stereo {

    SparseStereoProc::SparseStereoProc(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh)
    {
        params_ = std::make_shared<matching_params>();
        frame_counter_ = 0;
        restart_frame_counter_ = 0;
//         visualize_enable_ = true;
        
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
        
        std::string config_file;
        pnh_.getParam("config_file", config_file);
        pnh_.setParam("config_file", config_file);
        if (!config_file.empty()) stvo::Config::loadFromFile(config_file);

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

        tracked_features_pub_ = nh_.advertise<sparse_stereo_msgs::TrackedPointList>(name_ + "/tracked_features", 100, connect_cb_info, connect_cb_info);

        pub_points2_ = nh_.advertise<PointCloud>(name_ + "/sparse_points2", 1, connect_cb_info, connect_cb_info);
        pub_sparse_lines_ = nh_.advertise<visualization_msgs::Marker>(name_ + "/sparse_lines", 1, connect_cb_info, connect_cb_info);
        pub_depth_ = it_->advertiseCamera(name_ + "/sparse_depth", 1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
        pub_vis_matching_result_ = it_vis_->advertise(name_ + "/matching_result", 1, connect_cb, connect_cb);
        pub_vis_tracked_result_ = it_vis_tracked_->advertise(name_ + "/tracked_result", 1, connect_cb, connect_cb);

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
        timer_.start();
        
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
        
        l_image_ = cv_bridge::toCvShare(l_image_msg)->image;
        r_image_ = cv_bridge::toCvShare(r_image_msg)->image;

        if(frame_counter_ == 0)
        {
            cam_ = new PinholeStereoCamera(
                            l_info_msg->width,
                            l_info_msg->height,
                            fabs(l_info_msg->P[0]),
                            fabs(l_info_msg->P[5]),
                            l_info_msg->P[2],
                            l_info_msg->P[6],
                            r_info_msg->P[3] / -r_info_msg->P[0],
                            l_info_msg->D[0],
                            l_info_msg->D[1],
                            l_info_msg->D[2],
                            l_info_msg->D[3],
                            l_info_msg->D[4] );
            
            
            StVO_ = new StereoFrameHandler;
            StVO_->loadCam(cam_);
            
            StVO_->initialize(l_image_, r_image_, 0);
        }
        else if(StVO_->num_tracked_pt >= 0 && StVO_->num_tracked_pt <= 20)
        {
            StVO_ = new StereoFrameHandler;
            StVO_->loadCam(cam_);
            
            StVO_->initialize(l_image_, r_image_, 0);
            
            restart_frame_counter_++;
            ROS_INFO_STREAM_NAMED(name_, "Restart number: [ " << restart_frame_counter_ << " ]. Reinitialized features tracking after tracking [ " << frame_counter_ << " ] frames.");
            
            frame_counter_ = 0;
            
        }
        else
        {
            StVO_->insertStereoPair( l_image_, r_image_, frame_counter_ );
            
            ros::WallTime start = ros::WallTime::now();
            if(optimize_pose_enable_)
            {
                StVO_->optimizePose();
                ROS_INFO_STREAM_NAMED(name_, "Optimizing pose takes: " << (ros::WallTime::now() - start).toSec() << " ms.");
            }
        }
        double t_stvo = timer_.stop();
        timer_.start();
        
        // Visualize matched features
        if(visualize_enable_)
        {
            cv::Mat matched_img = StVO_->curr_frame->plotStereoFrame();
            sensor_msgs::ImagePtr matched_img_msg = cv_bridge::CvImage(l_image_msg->header, "bgr8", matched_img).toImageMsg();
            pub_vis_matching_result_.publish(matched_img_msg);
            
            cv::Mat tracked_img;
            StVO_->plotLeftPair(tracked_img);
//             tracked_img = StVO_->prev_frame->img_l.clone();
            sensor_msgs::ImagePtr tracked_img_msg = cv_bridge::CvImage(l_image_msg->header, "bgr8", tracked_img).toImageMsg();
            pub_vis_tracked_result_.publish(tracked_img_msg);
        }
        
        // Publishing point clouds and lines in 3d
        if(frame_counter_ == 0)
        {
            stereo_pt_ = StVO_->curr_frame->getStereoPt();
            stereo_ls_ = StVO_->curr_frame->getStereoLs();
        }
        else
        {
            stereo_pt_ = StVO_->matched_pt;
            stereo_ls_ = StVO_->matched_ls;
        }
        
        int tracked_pt = 0;
        if(StVO_->num_tracked_pt > -1)
        {
            tracked_pt = StVO_->num_tracked_pt;
        }
        
//         for(auto pt : stereo_pt_)
//         {
//             std::cout << StVO_->num_tracked_pt << std::endl;
//         }

        // Obtain the tracked points
        sparse_stereo_msgs::TrackedPointList tracked_points;
        tracked_points.header = l_image_msg->header;

        tracked_points.tracked_list.clear();
        for(auto pt : stereo_pt_)
        {
            if(pt->inlier && pt->idx != -1 && pt->P(2) >= 0.5)
            {
                sparse_stereo_msgs::TrackedPoint tracked_pt_t;
                tracked_pt_t.header = l_image_msg->header;

                tracked_pt_t.depth = pt->P(2);
                tracked_pt_t.id = pt->idx;
                tracked_pt_t.u_l = pt->pl(0);
                tracked_pt_t.v_l = pt->pl(1);

                tracked_points.tracked_list.push_back(tracked_pt_t);
            }
        }

        tracked_features_pub_.publish(tracked_points);

        ROS_INFO_STREAM_NAMED(name_, "Tracked [ " << tracked_points.tracked_list.size() << " ] features.");

        if(pub_points2_.getNumSubscribers() > 0 || pub_sparse_lines_.getNumSubscribers() > 0)
        {
            if (frame_counter_ == 0)
            {
                if (l_image_msg->header.frame_id == "camera_gray_left")
                {
                    points_msg_->header.frame_id = l_image_msg->header.frame_id;
                    lines_msg_->header.frame_id = l_image_msg->header.frame_id;
                }
                else
                {
                    points_msg_->header.frame_id = local_frame_;
                    lines_msg_->header.frame_id = local_frame_;
                    // points_msg->header.frame_id = "camera_depth_optical_frame";
                }
                
                points_msg_->height = 1;
                
                lines_msg_->ns = "sparse_lines";
                lines_msg_->action = visualization_msgs::Marker::ADD;
                lines_msg_->pose.orientation.w = 1.0;
                lines_msg_->id = 0;
                lines_msg_->type = visualization_msgs::Marker::LINE_LIST;
                lines_msg_->scale.x = 0.1;
                lines_msg_->color.r = 1.0;
                lines_msg_->color.a = 1.0;
            }
            points_msg_->points.clear();
            lines_msg_->points.clear();
            pcl_conversions::toPCL(l_image_msg->header.stamp, points_msg_->header.stamp);
            pcl_conversions::fromPCL(points_msg_->header.stamp, lines_msg_->header.stamp);
            
            sparse_depth_ = dNaN * cv::Mat::ones(l_image_.rows, l_image_.cols, CV_32FC1);   // depth image for sparse lines
            generate3DPtLi(cam_, stereo_pt_, stereo_ls_, points_msg_, lines_msg_, sparse_depth_);
            
            pub_points2_.publish(points_msg_);
            pub_sparse_lines_.publish(lines_msg_);
        }
        
        
        // publish sparse depth image
        if (pub_depth_.getNumSubscribers() > 0)
        {
            sensor_msgs::ImagePtr sparse_depth_msg = cv_bridge::CvImage(l_image_msg->header, sensor_msgs::image_encodings::TYPE_32FC1, sparse_depth_).toImageMsg();
            sparse_depth_msg->header.frame_id = local_frame_;

            sensor_msgs::CameraInfoPtr sparse_depth_info_msg = boost::make_shared<sensor_msgs::CameraInfo>(*l_info_msg);
            sparse_depth_info_msg->header.frame_id = local_frame_;
            sparse_depth_info_msg->height = sparse_depth_.rows;
            sparse_depth_info_msg->width = sparse_depth_.cols;

            pub_depth_.publish(sparse_depth_msg, sparse_depth_info_msg);
        }
        
        // update StVO
        if(frame_counter_ != 0) StVO_->updateFrame();
        
        
        frame_counter_++;
        double t_end = timer_.stop() + t_stvo;
        ROS_INFO_STREAM_NAMED(name_, "StVO took " << t_stvo << " ms.");
        ROS_INFO_STREAM_NAMED(name_, "Total process took " << t_end << " ms.");

    }
    
    void SparseStereoProc::generate3DPtLi(PinholeStereoCamera* cam, std::vector<PointFeature*>& stereo_pt, std::vector<LineFeature*>& stereo_ls, boost::shared_ptr<PointCloud>& points_msg, boost::shared_ptr<visualization_msgs::Marker>& lines_msg, cv::Mat& sparse_depth)
    {
        if(sparse_depth.channels() != 1)
            ROS_ERROR_STREAM_NAMED(name_, "Invalid depth image.");
        
        int pt_radius = 2;
        int line_thick = 3;
        
        // For points
        for(auto pt : stereo_pt)
        {
            if(pt->inlier && pt->P(2) >= 0.5)
            {
                pcl::PointXYZ point(pt->P(0), pt->P(1), pt->P(2));
                points_msg->points.push_back(point);
                
                // For sparse points depth image
                cv::Point center(pt->pl(0), pt->pl(1));
                cv::Scalar depth = pt->P(2);
                cv::circle(sparse_depth, center, pt_radius, depth, -1, cv::FILLED);
            }
        }
        points_msg->width = points_msg->points.size();
        
        
        // For lines
        int vertical_const = 5;
        int num_samples_scale = 50;
        
        for(auto ls : stereo_ls)
        {
            if(ls->inlier)
            {
                geometry_msgs::Point s_p;
                s_p.x = ls->sP(0);
                s_p.y = ls->sP(1);
                s_p.z = ls->sP(2);
                lines_msg->points.push_back(s_p);

                geometry_msgs::Point e_p;
                e_p.x = ls->eP(0);
                e_p.y = ls->eP(1);
                e_p.z = ls->eP(2);
                lines_msg->points.push_back(e_p);
                
                cv::Point3d sp(s_p.x, s_p.y, s_p.z);
                cv::Point3d ep(e_p.x, e_p.y, e_p.z);
                cv::Point3d direct_vec = ep - sp;
                double length = sqrt(pow(direct_vec.x, 2) + pow(direct_vec.y, 2) + pow(direct_vec.z, 2));
                int num_samples = int(length * num_samples_scale);
                
                // For sparse lines depth image
                double dis_vert_square = pow(e_p.x - s_p.x, 2) + pow(e_p.z - s_p.z, 2);
                if(dis_vert_square <= pow(vertical_const, 2))
                {
//                     cv::Point l_s = cv::Point(ls->spl(0), ls->spl(1));
//                     cv::Point l_e = cv::Point(ls->epl(0), ls->epl(1));
//                     cv::Point mid = (l_s + l_e) / 2;
//                     cv::Scalar depth_1 = s_p.z;
//                     cv::Scalar depth_2 = e_p.z;
//                     
//                     cv::line(sparse_depth_, l_s, mid, depth_1, line_thick);
//                     cv::line(sparse_depth_, mid, l_e, depth_2, line_thick);
                    
                    
                    // new method
                    for(double i = 0; i < num_samples+1; i++)
                    {
                        cv::Point3d sample = sp + i / num_samples * direct_vec;
                        cv::Point sample_uv = cam->projection(sample);
                        if(i != num_samples)
                        {
                            cv::Point3d sample_next = sp + (i + 1) /num_samples * direct_vec;
                            cv::Point sample_next_uv = cam->projection(sample_next);
                            cv::line(sparse_depth, sample_uv, sample_next_uv, sample.z, line_thick);
                        }
//                         cv::circle(sparse_depth, sample_uv, pt_radius, sample.z, -1, cv::FILLED);
                    }
                }
            }
        }
        
        return;
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

