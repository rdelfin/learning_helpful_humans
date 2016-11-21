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


class DatabaseImage {
public:
    DatabaseImage();
    DatabaseImage(boost::uuids::uuid identifier);
    DatabaseImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const sensor_msgs::PointCloud2& pointCloud);

    bool fetch();
    bool post();

    ~DatabaseImage();
private:
    boost::uuids::uuid identifier;

    sensor_msgs::Image imageData;
    ImageMetadata metadata;
    sensor_msgs::PointCloud2 pointCloud;
};


#endif //PROJECT_DATABASEIMAGE_HPP
