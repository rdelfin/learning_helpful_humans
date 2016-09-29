#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

ros::ServiceServer server;
std::string windowName = "Display Window";

bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ask_location_node");
    ros::NodeHandle nh;
    
    server = nh.advertiseService("ask_location", askQuestion);
    
    ros::spin();
    
    return 0;
}

bool askQuestion(bwi_msgs::ImageQuestionRequest& req, bwi_msgs::ImageQuestionResponse& res) {
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(req.image);
    
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::imshow(windowName, image->image);
    cv::waitKey(0);
}