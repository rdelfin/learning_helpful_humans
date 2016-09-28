#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>




int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ask_location_node_tester");
    ros::NodeHandle nh;
    
    bwi_msgs::ImageQuestionRequest req;
    bwi_msgs::ImageQuestionResponse res;
    
    ros::ServiceClient client = nh.serviceClient<bwi_msgs::ImageQuestion>("ask_location_node");
    
    ROS_INFO("Image path: %s", argv[1]);
    
    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    std_msgs::Header header;
    header.stamp = header.stamp.now();
    cv_bridge::CvImage bridgeImg(header, sensor_msgs::image_encodings::RGB16, image);
    req.image = *bridgeImg.toImageMsg();
    
    client.call(req, res);
    ROS_INFO("Client called");
    
    ros::spinOnce();
    
    return 0;
}
