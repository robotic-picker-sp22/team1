#include <vector>

#include "perception/crop.h"
#include "perception/downsampler.h"
#include "perception/segmentation.h"
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char **argv)
{
    if (argc < 2) {
        ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
        ros::spinOnce();
    }
    std::string data_dir(argv[1]);

    ros::init(argc, argv, "point_cloud_demo");
    ros::NodeHandle nh;
    ros::Publisher crop_pub =
        nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);
    ros::Subscriber sub_cropper =
        nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

    // for downsampling
    ros::Publisher ds_pub =
        nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
    perception::DownSampler sampler(ds_pub);
    ros::Subscriber ds_sub = nh.subscribe("cropped_cloud", 1, &perception::DownSampler::Callback, &sampler);

    // for segmentation
    ros::Publisher segment_pub = nh.advertise<sensor_msgs::PointCloud2>("segment_cloud", 1, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("segment_marker", 1, true);
    
    // add object recognition functionality
    // Create the object recognizer.
    std::vector<perception_msgs::ObjectFeatures> dataset;
    perception::LoadData(data_dir, &dataset);
    perception::ObjectRecognizer recognizer(dataset);

    perception::Segmenter segmenter(segment_pub, marker_pub, recognizer);
    ros::Subscriber sub_segmenter =
        nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
    std::cout << "444444444444444444444" << std::endl;
    ros::spin();
    std::cout << "HEYYYYYYYYYYYYYYYYY" << std::endl;
    return 0;
}