//
// Created by rdelfin on 10/26/16.
//

#include <ros/ros.h>

#include <bwi_msgs/UploadImage.h>
#include <bwi_msgs/GetNextImage.h>
#include <bwi_msgs/SaveImageResponse.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <cv_bridge/cv_bridge.h>

#include <learning_helpful_humans/imagetool/ImagePickerPolicy.h>
#include <learning_helpful_humans/imagetool/RandomImagePolicy.h>
#include <learning_helpful_humans/request/PostImageRequest.h>
#include <learning_helpful_humans/request/GetFieldValue.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <sensor_msgs/PointCloud.h>

bool uploadImageCb(bwi_msgs::UploadImageRequest& req, bwi_msgs::UploadImageResponse& res);
bool nextImageCb(bwi_msgs::GetNextImageRequest& req, bwi_msgs::GetNextImageResponse& res);
bool saveResponseCb(bwi_msgs::SaveImageResponseRequest& req, bwi_msgs::SaveImageResponseResponse& res);

bool uploadImage(std::string base_name, sensor_msgs::Image& img);

ImagePickerPolicy* policy;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_tool_node");
    ros::NodeHandle nh;

    policy = new RandomImagePolicy();

    ros::ServiceServer uploadImageServer = nh.advertiseService("image_tool/upload", uploadImageCb);
    ros::ServiceServer getNextImageServer = nh.advertiseService("image_tool/next", nextImageCb);
    ros::ServiceServer saveResponseServer = nh.advertiseService("image_tool/save_response", saveResponseCb);

    ros::spin();

    delete policy;
    return 0;
}

bool uploadImageCb(bwi_msgs::UploadImageRequest& req, bwi_msgs::UploadImageResponse& res) {
    boost::uuids::random_generator gen;
    std::string base_name = boost::uuids::to_string(gen());

    res.success = (unsigned char)uploadImage(base_name, req.img);   // Upload image only

    // On failure, abort and inform caller
    if(!res.success) {
        return true;
    }

    return true;
}


bool nextImageCb(bwi_msgs::GetNextImageRequest& req, bwi_msgs::GetNextImageResponse& res) {
    boost::uuids::uuid image_id = policy->getNextImage();
    DatabaseImage dbImage(image_id);
    dbImage.fetch();

    sensor_msgs::Image sensorImg = dbImage.getImageData();

    // PCL Point cloud conversion
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud = dbImage.getPointCloud();
    pcl::PCLPointCloud2 pointCloud2;
    sensor_msgs::PointCloud2 pointCloudRos;
    pcl::toPCLPointCloud2(pointCloud, pointCloud2);
    pcl_conversions::fromPCL(pointCloud2, pointCloudRos);


    res.base_name = boost::uuids::to_string(image_id);
    res.img = sensorImg;
    res.pc = pointCloudRos;

    return false;
}

bool saveResponseCb(bwi_msgs::SaveImageResponseRequest& req, bwi_msgs::SaveImageResponseResponse& res) {
    boost::uuids::uuid image_id = boost::lexical_cast<boost::uuids::uuid>(req.base_name);

    bool success;

    Answer answer;
    answer.location = req.location;
    answer.answer = req.response;
    answer.answered = (req.response == "");
    answer.questionId = req.question_id;
    answer.timestamp = req.timestamp.sec;
    answer.next = -1;

    DatabaseImage img(image_id);
    success = img.fetch();

    if(!success) {
        res.success = (unsigned char)false;
        return true;
    }

    img.addAnswer(answer);
    img.post();

    res.success = (unsigned char)success;
    return true;

}


bool uploadImage(std::string base_name, sensor_msgs::Image& img) {
    cv_bridge::CvImagePtr imageBridge = cv_bridge::toCvCopy(img);
    std::vector<uchar> dataBuffer;
    cv::imencode("jpg", imageBridge->image, dataBuffer); // Stores jpg data in dataBuffer

    PostImageRequest postReq(&dataBuffer[0], dataBuffer.size(), base_name, "image/jpeg"); // Create the HTTP request
    return postReq.perform();                               // Post request to firebase
}