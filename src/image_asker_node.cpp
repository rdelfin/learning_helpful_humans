#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>
#include <bwi_msgs/QuestionDialog.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

ros::ServiceServer server;
ros::ServiceClient questionClient;
std::string windowName = "Display Window";
std::string question = "How would you describe the image on the right?\n(type nothing in box and press enter to make me leave)";

cv_bridge::CvImagePtr img = nullptr;
std::mutex m;


bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);
void imgThread();

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_asker_node");
    ros::NodeHandle nh;
    
    server = nh.advertiseService("ask_location", askQuestion);
    questionClient = nh.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");

    ROS_INFO("Creating window...");
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    ROS_INFO("Starting second thread...");
    std::thread th(imgThread);
    
    ros::spin();

    th.join();
    
    return 0;
}

// TODO: Add go away nicely and not nicely, and answer

bool askQuestion(bwi_msgs::ImageQuestionRequest& req, bwi_msgs::ImageQuestionResponse& res) {
    ROS_INFO("Question request made");
    m.lock();
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    img = cv_bridge::toCvCopy(req.image);
    m.unlock();

    bwi_msgs::QuestionDialogRequest questionReq;
    bwi_msgs::QuestionDialogResponse questionRes;
    
    questionReq.message = question;
    questionReq.timeout = 30.0;
    questionReq.type = bwi_msgs::QuestionDialogRequest::TEXT_QUESTION;
    questionClient.call(questionReq, questionRes);

    m.lock();
    cv::destroyWindow(windowName);
    m.unlock();
    
    if(questionRes.text != "")
        res.answers.push_back(questionRes.text);
    
    return true;
}

void imgThread() {
    ROS_INFO("Second thread!");
    ros::Rate r(10);
    while(ros::ok()) {
        if(img != nullptr) {
            m.lock();
            cv::imshow(windowName, img->image);
            cv::waitKey(0);
            img = nullptr;
            m.unlock();
        }
        r.sleep();
    }
}