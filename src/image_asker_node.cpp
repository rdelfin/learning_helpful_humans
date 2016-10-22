#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>

#include <learning_helpful_humans/Fl_ViewerCV.h>

ros::ServiceServer server;
std::string question = "How would you describe the image above?";

cv_bridge::CvImagePtr img = nullptr;
std::mutex m;

int g_argc;
char** g_argv;

void btnGoAwayCallback(Fl_Widget* widget, void*);
void btnOkCallback(Fl_Widget* widget, void*);
void textboxCallback(Fl_Widget* widget, void*);
void windowCallback(Fl_Widget* widget, void*);
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


void textboxCallback(Fl_Widget* widget, void*) {
    std::cout << "Enter pressed" << std::endl;
}

void windowCallback(Fl_Widget* widget, void*) {
    std::cout << "Close pressed" << std::endl;
    ((Fl_Window*)widget)->hide();
}

void showQuestion() {
    Fl_Window *window = new Fl_Window(700,800);
    window->callback(windowCallback);

    // Question Label
    Fl_Box* questionBox = new Fl_Box(20, 625, 660, 20);
    questionBox->label(question.c_str());
    questionBox->labelsize(20);
    questionBox->callback();

    // Input Text Box
    Fl_Input* ansBox = new Fl_Input(50, 660, 600, 30);
    ansBox->textsize(20);
    ansBox->when(FL_WHEN_ENTER_KEY);
    ansBox->callback(textboxCallback);

    // Image
    Fl_ViewerCV* viewer = new Fl_ViewerCV(50, 10, 600, 600);
    viewer->SetImage(&img->image);

    // Ok Button
    Fl_Button* okBtn = new Fl_Button(20, 720, 100, 50, "Ok");
    okBtn->labelsize(20);
    okBtn->when(FL_WHEN_RELEASE);
    okBtn->callback(btnOkCallback);

    // Leave Button
    Fl_Button* leaveBtn = new Fl_Button(150, 720, 150, 50, "Please Leave");
    leaveBtn->labelsize(20);
    leaveBtn->when(FL_WHEN_RELEASE);
    leaveBtn->callback(btnGoAwayCallback);

    window->end();
    window->show(g_argc, g_argv);
    Fl::run();

    delete window;
}
