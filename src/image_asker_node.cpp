#include <ros/ros.h>
#include <bwi_msgs/ImageQuestion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>

ros::ServiceServer server;
//ros::ServiceClient questionClient;
std::string windowName = "Display Window";
std::string question = "How would you describe the image on the right?\n(type nothing in box and press enter to make me leave)";

cv_bridge::CvImagePtr img = nullptr;
std::mutex m;

int g_argc;
char** g_argv;

bool askQuestion(bwi_msgs::ImageQuestionRequest&, bwi_msgs::ImageQuestionResponse&);
void imgThread();

int main(int argc, char* argv[]) {
    g_argc = argc;
    g_argv = argv;
    ros::init(argc, argv, "image_asker_node");
    ros::NodeHandle nh;
    
    server = nh.advertiseService("ask_location", askQuestion);
    //questionClient = nh.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");

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
    img = cv_bridge::toCvCopy(req.image);
    
    return true;
}

void imgThread() {
    Fl_Window *window = new Fl_Window(400,180);
    Fl_Box *box = new Fl_Box(20,40,260,100,"Hello, World!");
    box->box(FL_UP_BOX);
    box->labelsize(36);
    box->labelfont(FL_BOLD+FL_ITALIC);
    box->labeltype(FL_SHADOW_LABEL);
    window->end();
    window->show(g_argc, g_argv);
    Fl::run();
}
