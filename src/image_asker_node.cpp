#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>

#include <learning_helpful_humans/Fl_ViewerCV.h>

ros::ServiceServer server;
//ros::ServiceClient questionClient;
std::string windowName = "Display Window";
std::string question = "How would you describe the image on the right?\n(type nothing in box and press enter to make me leave)";

cv_bridge::CvImagePtr img = nullptr;
std::mutex m;

int g_argc;
char** g_argv;

void btnGoAwayCallback();
void btnOkCallback();
void showQuestion();

bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);
void imgThread();

int main(int argc, char* argv[]) {
    g_argc = argc;
    g_argv = argv;
    ros::init(argc, argv, "image_asker_node");
    ros::NodeHandle nh;
    
    server = nh.advertiseService("ask_location", askQuestion);
    //questionClient = nh.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");

    ros::spin();
    
    return 0;
}

// TODO: Add go away nicely and not nicely, and answer

bool askQuestion(bwi_msgs::ImageQuestionRequest& req, bwi_msgs::ImageQuestionResponse& res) {
    ROS_INFO("Question request made");
    img = cv_bridge::toCvCopy(req.image);
    showQuestion();

    return true;
}

void btnOkCallback(Fl_Widget* widget, void*) {
    std::cout << "Ok pressed" << std::endl;
}

void btnGoAwayCallback(Fl_Widget* widget, void*) {
    std::cout << "Go away pressed" << std::endl;
}

void windowCallback(Fl_Widget* widget, void*) {
    std::cout << "Close pressed" << std::endl;
    ((Fl_Window*)widget)->hide();
}

void showQuestion() {
    Fl_Window *window = new Fl_Window(800,800);
    window->callback(windowCallback);

    Fl_ViewerCV* viewer = new Fl_ViewerCV(20, 20, 700, 700);
    viewer->SetImage(&img->image);


    Fl_Button* okBtn = new Fl_Button(20, 720, 100, 50, "Ok");
    Fl_Button* leaveBtn = new Fl_Button(150, 720, 150, 50, "Please Leave");

    okBtn->labelsize(20);
    leaveBtn->labelsize(20);

    okBtn->when(FL_WHEN_RELEASE);
    leaveBtn->when(FL_WHEN_RELEASE);
    okBtn->callback(btnOkCallback);
    leaveBtn->callback(btnGoAwayCallback);

    window->end();
    window->show(g_argc, g_argv);
    Fl::run();

    delete window;
}
