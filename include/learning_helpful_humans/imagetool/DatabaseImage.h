//
// Created by rdelfin on 11/11/16.
//

#ifndef PROJECT_DATABASEIMAGE_HPP
#define PROJECT_DATABASEIMAGE_HPP

#include <boost/uuid/uuid.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <learning_helpful_humans/imagetool/ImageMetadata.h>


#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>


class DatabaseImage {
public:
    DatabaseImage();
    DatabaseImage(boost::uuids::uuid identifier);

    /**
     * Creates an image to be posted from ImageData, metadata and pointcloud. It is recommended you call fetch afterwards
     * @param imageData The image data, in ROS format
     * @param metadata The metadata, including identifier. If you want to create a new image, the identifier will be ignored.
     * @param pointCloud A PCL point cloud of the location the image was taken in (usually recorded from kinect-like device)
     * @param newImage True if this is a new image and needs an identifier assigned. False otherwise
     */
    DatabaseImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, bool newImage = false);

    bool fetch();
    bool post();

    const sensor_msgs::Image& getImageData();
    const ImageMetadata& getMetadata();
    const pcl::PointCloud<pcl::PointXYZRGB>& getPointCloud();

    void addAnswer(Answer a);

    ~DatabaseImage();
private:
    boost::uuids::uuid identifier;

    sensor_msgs::Image imageData;
    ImageMetadata metadata;
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
};


#endif //PROJECT_DATABASEIMAGE_HPP
