#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>
#include <bwi_msgs/QuestionDialog.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

ros::ServiceServer server;
ros::ServiceClient questionClient;
std::string windowName = "Display Window";
std::string question = "How would you describe the image on the right?\n(type nothing in box and press enter to make me leave)";

bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_asker_node");
    ros::NodeHandle nh;
    
    server = nh.advertiseService("ask_location", askQuestion);
    questionClient = nh.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
    
    ros::spin();
    
    return 0;
}

// TODO: Add go away nicely and not nicely, and answer

bool askQuestion(bwi_msgs::ImageQuestionRequest& req, bwi_msgs::ImageQuestionResponse& res) {
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(req.image);
    
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::imshow(windowName, image->image);
    cv::waitKey(0);
    
    bwi_msgs::QuestionDialogRequest questionReq;
    bwi_msgs::QuestionDialogResponse questionRes;
    
    questionReq.message = question;
    questionReq.timeout = 30.0;
    questionReq.type = bwi_msgs::QuestionDialogRequest::TEXT_QUESTION;
    questionClient.call(questionReq, questionRes);
    
    if(questionRes.text != "")
        res.answers.push_back(questionRes.text);
    
    return true;
}