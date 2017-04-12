/**
  * Created by Ricardo Delfin Garcia on 10/26/16.
  *
  * This node provides a set of services used to interact with the images on the firebase (online) database.
  * It provides `image_tool/upload` to upload new images and update old ones on firebase. It also provides
  * `image_tool/next` to fetch a random image using some internal heuristic. In the case that a UUID is
  * provided, it will fetch the image with said UUID. Finally, it provides  `image_tool/save_response` to
  * save an answer given by a user to an image.
  */

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
#include <learning_helpful_humans/offline/ImageCache.h>

#include <opencv2/opencv.hpp>

#include <signal.h>

#include <vector>
#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <sensor_msgs/PointCloud.h>

#include <curlpp/cURLpp.hpp>

#define CACHED_IMAGES (10)


bool uploadImageCb(bwi_msgs::UploadImageRequest& req, bwi_msgs::UploadImageResponse& res);
bool nextImageCb(bwi_msgs::GetNextImageRequest& req, bwi_msgs::GetNextImageResponse& res);
bool saveResponseCb(bwi_msgs::SaveImageResponseRequest& req, bwi_msgs::SaveImageResponseResponse& res);

bool uploadImage(std::string base_name, sensor_msgs::Image& img);

ImagePickerPolicy* policy;
ImageCache* cache;

int main(int argc, char* argv[]) {
    // Setup cURL to accept multiple threads
    signal(SIGPIPE, SIG_IGN);
    
    ros::init(argc, argv, "image_tool_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    ros::Rate r(10);

    policy = new RandomImagePolicy();
    cache = new ImageCache(policy, CACHED_IMAGES);

    ros::ServiceServer uploadImageServer = nh.advertiseService("image_tool/upload", uploadImageCb);
    ros::ServiceServer getNextImageServer = nh.advertiseService("image_tool/next", nextImageCb);
    ros::ServiceServer saveResponseServer = nh.advertiseService("image_tool/save_response", saveResponseCb);

    spinner.start();
    
    while(ros::ok()) {
        cache->update();
        r.sleep();
    }
    

    delete policy;
    return 0;
}

bool uploadImageCb(bwi_msgs::UploadImageRequest& req, bwi_msgs::UploadImageResponse& res) {
    ImageMetadata metadata;
    metadata.pose = req.pose;

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pcl::fromROSMsg(req.pc, pointCloud);

    return cache->addNewImage(req.img, metadata, pointCloud);
}


bool nextImageCb(bwi_msgs::GetNextImageRequest& req, bwi_msgs::GetNextImageResponse& res) {
    ROS_INFO_STREAM("Next image requested");
    DatabaseImage dbImage = cache->getNextImage();
    ROS_INFO_STREAM("Image ID: " << boost::uuids::to_string(dbImage.getIdentifier()));

    ImageMetadata metadata = dbImage.getMetadata();
    ROS_DEBUG_STREAM("Image metadata:");
    ROS_DEBUG_STREAM("\tPose: [x:" << metadata.pose.position.x << ", y:" << metadata.pose.position.y << ", z:" << metadata.pose.position.z << "]");
    ROS_DEBUG_STREAM("\tAnswers:");
    for(size_t i = 0; i < metadata.answers.size(); i++) {
        ROS_DEBUG_STREAM("\t\t[" << i << "]: " << metadata.answers[i].answer);
    }
    ROS_DEBUG_STREAM("Image dimensions: [" << dbImage.getImageData().width << "x" << dbImage.getImageData().height << "]");

    sensor_msgs::Image sensorImg = dbImage.getImageData();

    // PCL Point cloud conversion
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud = dbImage.getPointCloud();
    pcl::PCLPointCloud2 pointCloud2;
    sensor_msgs::PointCloud2 pointCloudRos;
    pcl::toPCLPointCloud2(pointCloud, pointCloud2);
    pcl_conversions::fromPCL(pointCloud2, pointCloudRos);

    res.pose = dbImage.getMetadata().pose;
    res.base_name = boost::uuids::to_string(dbImage.getIdentifier());
    res.img = sensorImg;
    res.pc = pointCloudRos;

    return true;
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
    
    res.success = cache->addAnswer(image_id, answer);
    return true;

}
