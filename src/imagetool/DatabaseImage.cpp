//
// Created by rdelfin on 11/11/16.
//

#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <learning_helpful_humans/request/PostImageRequest.h>
#include <learning_helpful_humans/request/AppendFieldValue.h>
#include <learning_helpful_humans/request/GetFieldValue.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <boost/uuid/uuid_io.hpp>
#include <learning_helpful_humans/request/GetImage.h>
#include <tf/transform_datatypes.h>


DatabaseImage::DatabaseImage() {

}

DatabaseImage::DatabaseImage(boost::uuids::uuid identifier)
    : identifier(identifier) {

}

DatabaseImage::DatabaseImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const sensor_msgs::PointCloud2& pointCloud)
    : imageData(imageData), metadata(metadata), pointCloud(pointCloud) {

}

bool DatabaseImage::fetch() {
    // Fetch image
    GetImage getImg(identifier, "jpg");

    cv::Mat img = getImg.performImage();
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    imageData = *cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

    // Fetch pose
    std::stringstream pathString;
    pathString << boost::uuids::to_string(identifier) << "/pose.json";
    GetFieldValue poseGet(pathString.str());
    json poseJson = poseGet.performAsJson();
}


bool DatabaseImage::post() {
    bool success;

    std::string basename = boost::uuids::to_string(identifier);

    cv_bridge::CvImagePtr imageBridge = cv_bridge::toCvCopy(imageData);
    std::vector<uchar> dataBuffer;
    cv::imencode("jpg", imageBridge->image, dataBuffer); // Stores jpg data in dataBuffer

    PostImageRequest imagePost(&dataBuffer[0], dataBuffer.size(), basename, "image/jpeg"); // Create the HTTP request
    success = imagePost.perform();                               // Post request to firebase

    // TODO: Post point cloud image somehow

    // Check for failure and abort
    if(!success)
        return false;

    success = metadata.postUpdate();

    return success;
}

DatabaseImage::~DatabaseImage() {

}
