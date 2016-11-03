//
// Created by rdelfin on 10/26/16.
//

#include <ros/ros.h>
#include <bwi_msgs/UploadImage.h>
#include <bwi_msgs/GetNextImage.h>
#include <bwi_msgs/SaveImageResponse.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

bool uploadImageCb(bwi_msgs::UploadImageRequest& req, bwi_msgs::UploadImageResponse& res);
bool nextImageCb(bwi_msgs::GetNextImageRequest& req, bwi_msgs::GetNextImageResponse& res);
bool saveResponseCb(bwi_msgs::SaveImageResponseRequest& req, bwi_msgs::SaveImageResponseResponse& res);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_tool_node");
    ros::NodeHandle nh;

    ros::ServiceServer uploadImageServer = nh.advertiseService("image_tool/upload", uploadImageCb);
    ros::ServiceServer getNextImageServer = nh.advertiseService("image_tool/next", nextImageCb);
    ros::ServiceServer saveResponseServer = nh.advertiseService("image_tool/save_response", saveResponseCb);

    ros::spin();

    return 0;
}

bool uploadImageCb(bwi_msgs::UploadImageRequest& req, bwi_msgs::UploadImageResponse& res) {
    
    return true;
}


bool nextImageCb(bwi_msgs::GetNextImageRequest& req, bwi_msgs::GetNextImageResponse& res) {

}

bool saveResponseCb(bwi_msgs::SaveImageResponseRequest& req, bwi_msgs::SaveImageResponseResponse& res) {

}