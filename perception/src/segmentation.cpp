#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include <pcl/common/common.h>
#include "pcl/filters/extract_indices.h"
#include "perception/object.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <sstream>
#include "perception/object_recognizer.h"

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception
{
    Segmenter::Segmenter(const ros::Publisher &points_pub, const ros::Publisher &marker_pub,
                         const ObjectRecognizer& recognizer)
        : points_pub_(points_pub), marker_pub_(marker_pub), recognizer_(recognizer) {}

    void Segmenter::Callback(const sensor_msgs::PointCloud2 &msg)
    {
        PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud_unfiltered);
        PointCloudC::Ptr cloud(new PointCloudC());
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

        // std::vector<pcl::PointIndices> object_indices;
        // SegmentBinObjects(cloud, &object_indices);

        std::vector<Object> objects;
        SegmentObjects(cloud, &objects);
        std::cout << "HEYYYYYYYYYYYYYYYYY" << std::endl;

        for (size_t i = 0; i < objects.size(); ++i)
        {
            Object& object = objects[i];

            // Recognize the object.
            std::string name;
            double confidence;
            recognizer_.Recognize(object, &name, &confidence);
            confidence = round(1000 * confidence) / 1000;

            object.name = name;
            object.confidence = confidence;

            // Publish a bounding box around it.
            visualization_msgs::Marker object_marker;
            object_marker.ns = "objects";
            object_marker.id = i;
            object_marker.header.frame_id = "base_link";
            object_marker.type = visualization_msgs::Marker::CUBE;
            object_marker.pose = object.pose;
            object_marker.scale = object.dimensions;
            object_marker.color.g = 1;
            object_marker.color.a = 0.3;
            std::cout << "Object name is:" << object.pose.position.x << std::endl;
            marker_pub_.publish(object_marker);

            std::stringstream ss;
            ss << object.name << " (" << object.confidence << ")";

            // Publish the recognition result.
            visualization_msgs::Marker name_marker;
            name_marker.ns = "recognition";
            name_marker.id = i;
            name_marker.header.frame_id = "base_link";
            name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            name_marker.pose.position = object.pose.position;
            name_marker.pose.position.z += 0.1;
            name_marker.pose.orientation.w = 1;
            name_marker.scale.x = 0.025;
            name_marker.scale.y = 0.025;
            name_marker.scale.z = 0.025;
            name_marker.color.r = 0;
            name_marker.color.g = 0;
            name_marker.color.b = 1.0;
            name_marker.color.a = 1.0;
            name_marker.text = ss.str();
            marker_pub_.publish(name_marker);
        }
    }

    void SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                      std::vector<pcl::PointIndices> *indices)
    {
        Euclid(cloud, indices);
        // RegionGrowing(cloud, indices);
        // ColorRegionGrowing(cloud, indices);
        // Find the size of the smallest and the largest object,
        // where size = number of points in the cluster
        size_t min_size = std::numeric_limits<size_t>::max();
        size_t max_size = std::numeric_limits<size_t>::min();
        for (size_t i = 0; i < indices->size(); ++i)
        {
            size_t cluster_size = (*indices)[i].indices.size();
            min_size = std::min(cluster_size, min_size);
            max_size = std::max(cluster_size, max_size);
        }

        ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
                 indices->size(), min_size, max_size);
    }

    void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                              geometry_msgs::Pose *pose,
                                              geometry_msgs::Vector3 *dimensions)
    {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);

        pose->position.x = (max_pt.x() + min_pt.x()) / 2;
        pose->position.y = (max_pt.y() + min_pt.y()) / 2;
        pose->position.z = (max_pt.z() + min_pt.z()) / 2;
        pose->orientation.w = 1;

        dimensions->x = max_pt.x() - min_pt.x();
        dimensions->y = max_pt.y() - min_pt.y();
        dimensions->z = max_pt.z() - min_pt.z();
    }

    // Does a complete bin segmentation pipeline.
    //
    // Args:
    //  cloud: The point cloud with the bin and the objects in it.
    //  objects: The output objects.
    void SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   std::vector<Object> *objects)
    {
        std::vector<pcl::PointIndices> object_indices;
        SegmentBinObjects(cloud, &object_indices);
        for (size_t i = 0; i < object_indices.size(); ++i)
        {
            // Reify indices into a point cloud of the object.
            pcl::PointIndices::Ptr indices(new pcl::PointIndices);
            *indices = object_indices[i];
            PointCloudC::Ptr object_cloud(new PointCloudC());
            pcl::ExtractIndices<PointC> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(indices);
            extract.filter(*object_cloud);

            // Create an object from the object cloud.
            Object object = Object();
            GetAxisAlignedBoundingBox(object_cloud, &object.pose, &object.dimensions);
            object.cloud = object_cloud;

            objects->push_back(object);
        }
    }

    void Euclid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           std::vector<pcl::PointIndices> *indices)
    {
        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
        ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
        ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
        pcl::EuclideanClusterExtraction<PointC> euclid;
        euclid.setInputCloud(cloud);
        euclid.setClusterTolerance(cluster_tolerance);
        euclid.setMinClusterSize(min_cluster_size);
        euclid.setMaxClusterSize(max_cluster_size);
        euclid.extract(*indices);
    }

    void RegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices> *indices)
    {
        int min_cluster_size, max_cluster_size, num_of_neighbors, k_search;
        double smoothness_threshold, curvature_threshold;
        ros::param::param("reg_min_cluster_size", min_cluster_size, 10);
        ros::param::param("reg_max_cluster_size", max_cluster_size, 10000);
        ros::param::param("reg_k_search", k_search, 50);
        ros::param::param("reg_num_of_neighbors", num_of_neighbors, 30);
        ros::param::param("reg_smoothness_threshold", smoothness_threshold, 3.0);
        ros::param::param("reg_curvature_threshold", curvature_threshold, 1.0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *cloud_xyz);

        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud_xyz);
        normal_estimator.setKSearch(k_search);
        normal_estimator.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(num_of_neighbors);
        reg.setInputCloud(cloud_xyz);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothness_threshold / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvature_threshold);

        reg.extract(*indices);
    }

    void ColorRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices> *indices)
    {
        int min_cluster_size, max_cluster_size, distance_threshold, point_color_thrshold, region_color_threshold;
        ros::param::param("reg_min_cluster_size", min_cluster_size, 10);
        ros::param::param("reg_max_cluster_size", max_cluster_size, 10000);
        ros::param::param("reg_distance_threshold", distance_threshold, 10);
        ros::param::param("reg_point_color_thrshold", point_color_thrshold, 6);
        ros::param::param("reg_region_color_threshold", region_color_threshold, 5);

        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setInputCloud(cloud);
        reg.setDistanceThreshold(distance_threshold);
        reg.setPointColorThreshold(point_color_thrshold);
        reg.setRegionColorThreshold(region_color_threshold);
        reg.extract(*indices);
    }

} // namespace perception