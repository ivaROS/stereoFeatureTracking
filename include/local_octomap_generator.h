#ifndef _LOCAL_OCTOMAP_GENERATOR_
#define _LOCAL_OCTOMAP_GENERATOR_

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/OctomapServer.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/UInt32.h>

namespace pips_sparse_stereo {
namespace local_octomap
{
    typedef sensor_msgs::PointCloud2 ROSPointCloud;
    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
    typedef octomap::OcTree OCTree;

    class PointCloudOctomap 
    {
    public:

        static std::shared_ptr<OCTree> toOcTree(const PCLPointCloud& cloud, double m_maxRange = -1, double m_res=.05, double probHit=.7, double probMiss=.4, double thresMin=.12, double thresMax=.97);


        static visualization_msgs::MarkerArray visualizeOctomap(const std::shared_ptr<OCTree>& m_octree, const std_msgs::Header& header, const std::string& ns);
        // static visualization_msgs::MarkerArray visualizeOctomap(const std::shared_ptr<OCTree>& m_octree, const std_msgs::Header& header);

        inline
        static std::shared_ptr<octomap::OcTree> getOcTree(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
        {
            octomap::OcTree* tree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*octomap_msg);
            std::shared_ptr<octomap::OcTree> tree_ptr = std::make_shared<octomap::OcTree>(*tree);
            delete tree;
            return tree_ptr;
        }


    protected:
        inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
            for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
        };

        inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
            for (unsigned i = 0; i < 3; ++i)
            max[i] = std::max(in[i], max[i]);
        };

    };

    class OctomapGenerator
    {
    public:
        OctomapGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        bool init();

    private:
        void pointcloudCB(const ROSPointCloud::ConstPtr& point_cloud_msg);
        static std::shared_ptr<octomap::OcTree> getOcTree(const ROSPointCloud::ConstPtr& point_cloud_msg);

        void publishOctree(const std::shared_ptr<octomap::OcTree>& m_octree, const std_msgs::Header& header );

    private:
        std::string name_ = "octomap_generator";
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber pointcloud_sub_;
        PointCloudOctomap pointcloud_octree_;
        ros::Publisher pub_;

        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    };

}
}

#endif //_LOCAL_OCTOMAP_GENERATOR_
