//
// Created by rdelfin on 12/9/16.
//

#include <ros/ros.h>
#include <learning_helpful_humans/imagetool/DatabaseImage.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <FL/Fl_Window.H>
#include <learning_helpful_humans/Fl_ViewerCV.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_fetch_tester");
    ros::NodeHandle nh;

    boost::uuids::uuid imageId = boost::lexical_cast<boost::uuids::uuid>(std::string(argv[1]));
    DatabaseImage image(imageId);
    image.fetch();

    cv_bridge::CvImagePtr cvBridgeImage = cv_bridge::toCvCopy(image.getImageData());

    Fl_Window g_window(700,700);
    Fl_ViewerCV viewer(50, 10, 500, 500);
    viewer.SetImage(&cvBridgeImage->image);

    g_window.end();

    g_window.show(argc, argv);
    Fl::run();

//    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
//    cv::imshow( "Display window", cvBridgeImage->image);
//
//    cv::waitKey(0);
}
