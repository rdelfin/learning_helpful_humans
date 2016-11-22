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
    DatabaseImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud);

    bool fetch();
    bool post();

    ~DatabaseImage();
private:
    boost::uuids::uuid identifier;

    sensor_msgs::Image imageData;
    ImageMetadata metadata;
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
};


#endif //PROJECT_DATABASEIMAGE_HPP
