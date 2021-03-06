#include "local_octomap_generator.h"

using namespace octomap;

namespace pips_sparse_stereo {
namespace local_octomap
{
    
    std::shared_ptr<OCTree> PointCloudOctomap::toOcTree(const PCLPointCloud& cloud,  double m_maxRange, double m_res, double probHit, double probMiss, double thresMin, double thresMax)
    {
        // initialize octomap object & params
        std::shared_ptr<OCTree> m_octree = std::make_shared<OCTree>(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);

        point3d sensorOrigin(0,0,0);

        octomap::KeyRay m_keyRay;  // temp storage for ray casting
        KeySet free_cells, occupied_cells;


        // all other points: free on ray, occupied on endpoint:
        for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
            point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
                // occupied endpoint
                OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)){
                    occupied_cells.insert(key);
                }
            } else {// ray longer than maxrange:;
                point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
            }
        }

        // mark free cells only if not seen occupied in this cloud
        for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
            if (occupied_cells.find(*it) == occupied_cells.end()){
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        return m_octree;
    }

    visualization_msgs::MarkerArray PointCloudOctomap::visualizeOctomap(const std::shared_ptr<OCTree>& m_octree, const std_msgs::Header& header, const std::string& ns)
    {
        // init markers:
        visualization_msgs::MarkerArray occupiedNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVis.markers.resize(m_octree->getTreeDepth()+1);

        // now, traverse all leafs in the tree:
        for (octomap_server::OctomapServer::OcTreeT::iterator it = m_octree->begin(m_octree->getTreeDepth()), end = m_octree->end(); it != end; ++it)
        {
            if (m_octree->isNodeOccupied(*it))
            {
                double z = it.getZ();

                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();


                unsigned idx = it.getDepth();
                assert(idx < occupiedNodesVis.markers.size());

                geometry_msgs::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            }

        }


        for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
        {
            double size = m_octree->getNodeSize(i);

            occupiedNodesVis.markers[i].header.frame_id = header.frame_id;
            occupiedNodesVis.markers[i].header.stamp = header.stamp;
            occupiedNodesVis.markers[i].ns = ns;
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].scale.x = size;
            occupiedNodesVis.markers[i].scale.y = size;
            occupiedNodesVis.markers[i].scale.z = size;
            occupiedNodesVis.markers[i].color.g = 1;
            occupiedNodesVis.markers[i].color.a = .5;


            if (occupiedNodesVis.markers[i].points.size() > 0)
                occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            else
                occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        return occupiedNodesVis;
    }

    OctomapGenerator::OctomapGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh)
    {
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    }
    
    // Technically, there is probably no real reason to use shared pointers for most of these

    bool OctomapGenerator::init()
    {
        pointcloud_sub_ = nh_.subscribe("/sparse_stereo/sparse_points2", 10, &OctomapGenerator::pointcloudCB, this);
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sparse_stereo/octomap",10);
        // pub_ = nh_.advertise<std_msgs::UInt32>("/octomap",1);
    }
    

    void OctomapGenerator::pointcloudCB(const ROSPointCloud::ConstPtr& point_cloud_msg)
    {
        std::shared_ptr<octomap::OcTree> tree = getOcTree(point_cloud_msg);
        ROS_INFO_STREAM_NAMED(name_, "Octomap generated.");

        publishOctree(tree, point_cloud_msg->header);
    }

    std::shared_ptr<octomap::OcTree> OctomapGenerator::getOcTree(const ROSPointCloud::ConstPtr& point_cloud_msg)
    {
        PCLPointCloud::Ptr cloud (new PCLPointCloud);
        PCLPointCloud::Ptr new_cloud (new PCLPointCloud);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud, *new_cloud, mapping);

        std::shared_ptr<octomap::OcTree> tree = PointCloudOctomap::toOcTree(*new_cloud);
        
        return tree;
    }

    void OctomapGenerator::publishOctree(const std::shared_ptr<octomap::OcTree>& m_octree, const std_msgs::Header& header )
    {
        // init markers:
        visualization_msgs::MarkerArray occupiedNodesVis = PointCloudOctomap::visualizeOctomap(m_octree, header, "self_octomap");
        // visualization_msgs::MarkerArray occupiedNodesVis = PointCloudOctomap::visualizeOctomap(m_octree, header);
        pub_.publish(occupiedNodesVis);

        // std_msgs::UInt32 a;
        // a.data = 1;
        // pub_.publish(a);
    }

}
}

