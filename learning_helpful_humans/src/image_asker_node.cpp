/**
 * This node provides the GUI window used for asking the questions to the user. When a message is sent to
 * the `ask_location` service, the window pops up witht the appropriate image and question. The response is
 * sent back through the service.
 */

#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

#include <learning_helpful_humans/request/GetFieldValue.h>
#include <bwi_msgs/QuestionDialog.h>
#include <ros/callback_queue.h>

#define WINDOW "IMG_WINDOW"

ros::ServiceServer server;
ros::ServiceClient questionClient;

cv_bridge::CvImagePtr img = nullptr;

std::mutex imgMutex;

bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);

void imageUpdate(const ros::TimerEvent&);
std::pair<std::string, int> showQuestion(const std::string& question, long timeout);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_asker_node");

    ros::CallbackQueue cbqueue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&cbqueue);

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    questionClient = nh.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
    questionClient.waitForExistence();

    server = nh.advertiseService("ask_location", askQuestion);
    ros::Timer imageTimer = nh.createTimer(ros::Duration(0.1), imageUpdate);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin(&cbqueue);

    cv::destroyWindow(WINDOW);

    return 0;
}

bool askQuestion(bwi_msgs::ImageQuestionRequest& req, bwi_msgs::ImageQuestionResponse& res) {
    ROS_INFO("Question request made. Timeout: %ld", req.timeout);
    img = cv_bridge::toCvCopy(req.image);
    std::pair<std::string, int> answer = showQuestion(req.question, req.timeout);

    std::string status = "";
    switch(answer.second) {
        case 0: status = "RESPONDED"; break;
        case 1: status = "TIMEOUT"; break;
        case 2: status = "CANCELLED"; break;
        default: status = "UKNOWN STATUS";
    }

    ROS_INFO("Question finished. Status: %s. Text: '%s'", status.c_str(), answer.first.c_str());

    if(answer.first != "")
        res.answers.push_back(answer.first);

    switch(answer.second) {
        case 0:  res.end_reason = res.SUCCESSFUL_ANSWER; break;
        case 1:  res.end_reason = res.TIMEOUT; break;
        case 2:  res.end_reason = res.CANCELLED; break;
        default: res.end_reason = res.SUCCESSFUL_ANSWER;
    }

    return true;
}

void imageUpdate(const ros::TimerEvent& timer) {
    if(img != nullptr) {
        cv::imshow(WINDOW, img->image);
        cv::waitKey(3);
    }
}

// int: 0 for ok, 1 for timeout, 2 for cancelation
std::pair<std::string, int> showQuestion(const std::string& question, long timeout) {
    std::string answer;
    bwi_msgs::QuestionDialogRequest questionReq;
    bwi_msgs::QuestionDialogResponse questionRes;

    questionReq.message = "Could you help me answer a quick question?";
    questionReq.timeout = timeout;
    questionReq.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
    questionReq.options = {"Yes, what question?", "No, please leave"};
    questionClient.call(questionReq, questionRes);

    if(questionRes.index != bwi_msgs::QuestionDialogRequest::TEXT_RESPONSE &&
      (questionRes.index < 0 || questionRes.index == 1))
        return {"", questionRes.index == bwi_msgs::QuestionDialogRequest::TIMED_OUT ? 1 : 2};

    questionRes.index = 0;
    questionRes.text = "";
    questionReq.message = question;
    questionReq.timeout = 60.0;
    questionReq.type = bwi_msgs::QuestionDialogRequest::TEXT_QUESTION;
    questionClient.call(questionReq, questionRes);
    answer = questionRes.index < 0 && questionRes.index != -3 ? "" : questionRes.text;

    // Assume no timeout here. If they said ok and walked away, it is equivalent to cancelling
    int answer_idx = questionRes.index < 0 && questionRes.index != bwi_msgs::QuestionDialogRequest::TEXT_RESPONSE
                     ? 2 : 0;

    questionReq.message = "Thank you!";
    questionReq.type = bwi_msgs::QuestionDialogRequest::DISPLAY;
    questionReq.timeout = 0.1;                                  // (almost) immediate timeout
    questionClient.call(questionReq, questionRes);

    return {answer, answer_idx};
}
