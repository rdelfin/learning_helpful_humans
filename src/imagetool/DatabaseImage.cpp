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

char imageFormatString[] = R"(
{
    "%s": {
        "questions": {
            "count": 0,
            "list": []
        },
        "pose": {
            "x": %f,
            "y": %f,
            "z": %f,
            "rx": %f,
            "ry": %f,
            "rz": %f,
            "rw": %f
        }
    }
}
)";

DatabaseImage::DatabaseImage() {

}

DatabaseImage::DatabaseImage(boost::uuids::uuid identifier)
    : identifier(identifier) {

}

DatabaseImage::DatabaseImage(const sensor_msgs::Image& imageData, geometry_msgs::Pose pose, const sensor_msgs::PointCloud2& pointCloud)
    : imageData(imageData), pose(pose), pointCloud(pointCloud) {

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

    pose.position.x = poseJson["x"];
    pose.position.y = poseJson["y"];
    pose.position.z = poseJson["z"];
    pose.orientation.x = poseJson["rx"];
    pose.orientation.y = poseJson["ry"];
    pose.orientation.z = poseJson["rz"];
    pose.orientation.w = poseJson["rw"];
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



    char buf[1024];
    snprintf(buf, 1024, imageFormatString, boost::uuids::to_string(identifier),
             pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
             pose.orientation.y, pose.orientation.z, pose.orientation.w);

    json imageData = json::parse(std::string(buf));

    AppendFieldValue appendImageData("imagedata", imageData);
    success = appendImageData.perform();

    return success;
}

DatabaseImage::~DatabaseImage() {

}
