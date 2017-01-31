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

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>

#include <learning_helpful_humans/Fl_ViewerCV.h>

#include <learning_helpful_humans/request/GetFieldValue.h>

ros::ServiceServer server;

cv_bridge::CvImagePtr img = nullptr;
std::mutex m;

Fl_Window* g_window;
Fl_Input* g_ansBox;
Fl_Button *okBtn, *leaveBtn;
Fl_Box *questionBox;
Fl_ViewerCV* viewer;

std::string textbox;

int g_argc;
char** g_argv;

void btnGoAwayCallback(Fl_Widget* widget, void*);
void btnOkCallback(Fl_Widget* widget, void*);
void textboxCallback(Fl_Widget* widget, void*);
void windowCallback(Fl_Widget* widget, void*);
void showQuestion(const std::string&);


bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);
void imgThread();
void endQuestion(const std::string& answer);

bool asking;
bwi_msgs::ImageQuestionRequest g_req;
ros::Time questionStart;

int main(int argc, char* argv[]) {
    g_window = nullptr;
    g_ansBox = nullptr;
    okBtn = nullptr;
    leaveBtn = nullptr;
    questionBox = nullptr;
    viewer = nullptr;

    g_argc = argc;
    g_argv = argv;
    ros::init(argc, argv, "image_asker_node");
    ros::NodeHandle nh;

    server = nh.advertiseService("ask_location", askQuestion);

    ros::Rate r(10);
    while(ros::ok()) {
        if(asking) {
            ros::Duration timeDiff = ros::Time::now() - questionStart;

            if(timeDiff.toSec() > g_req.timeout) {
                endQuestion("");
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

bool askQuestion(bwi_msgs::ImageQuestionRequest& req, bwi_msgs::ImageQuestionResponse& res) {
    ROS_INFO("Question request made");
    g_req = req;
    img = cv_bridge::toCvCopy(req.image);
    showQuestion(req.question);

    if(textbox != "")
        res.answers.push_back(textbox);

    return true;
}

void endQuestion(const std::string& answer) {
    textbox = answer;
    okBtn->deactivate();
    leaveBtn->deactivate();
    g_ansBox->deactivate();

    g_window->hide();
}

void btnOkCallback(Fl_Widget* widget, void*) {
    std::cout << "Ok pressed" << std::endl;
    endQuestion(std::string(g_ansBox->value()));
}

void btnGoAwayCallback(Fl_Widget* widget, void*) {
    std::cout << "Go away pressed" << std::endl;
    endQuestion("");
}


void textboxCallback(Fl_Widget* widget, void*) {
    std::cout << "Enter pressed" << std::endl;
    endQuestion(std::string(g_ansBox->value()));
}

void windowCallback(Fl_Widget* widget, void*) {
    std::cout << "Close pressed" << std::endl;
    endQuestion("");
}

void showQuestion(const std::string& question) {
    // Clear out textbox
    textbox = "";

    if(g_window != nullptr)
        delete g_window;

    g_window = new Fl_Window(700,700);
    g_window->callback(windowCallback);

    // Question Label
    if(questionBox != nullptr)
        delete questionBox;
    questionBox = new Fl_Box(20, 525, 660, 20);
    questionBox->label(question.c_str());
    questionBox->labelsize(20);

    // Input Text Box
    if(g_ansBox != nullptr)
        delete g_ansBox;
    g_ansBox = new Fl_Input(50, 560, 600, 30);
    g_ansBox->textsize(20);
    g_ansBox->when(FL_WHEN_ENTER_KEY);
    g_ansBox->callback(textboxCallback);

    // Image
    if(viewer != nullptr)
        delete viewer;
    viewer = new Fl_ViewerCV(50, 10, 500, 500);
    viewer->SetImage(&img->image);

    // Ok Button
    if(okBtn != nullptr)
        delete okBtn;
    okBtn = new Fl_Button(20, 620, 100, 50, "Ok");
    okBtn->labelsize(20);
    okBtn->when(FL_WHEN_RELEASE);
    okBtn->callback(btnOkCallback);

    // Leave Button
    if(leaveBtn != nullptr)
        delete leaveBtn;
    leaveBtn = new Fl_Button(150, 620, 150, 50, "Please Leave");
    leaveBtn->labelsize(20);
    leaveBtn->when(FL_WHEN_RELEASE);
    leaveBtn->callback(btnGoAwayCallback);

    g_window->end();

    g_window->show(g_argc, g_argv);
    questionStart = ros::Time::now();
    asking = true;
    Fl::run();
}
