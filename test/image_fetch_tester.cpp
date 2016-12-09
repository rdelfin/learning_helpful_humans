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

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_fetch_tester");
    ros::NodeHandle nh;

    boost::uuids::uuid imageId = boost::lexical_cast<boost::uuids::uuid>(std::string(argv[1]));
    DatabaseImage image(imageId);
    image.fetch();

    cv_bridge::CvImagePtr cvBridgeImage = cv_bridge::toCvCopy(image.getImageData());

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow( "Display window", cvBridgeImage->image);

    cv::waitKey(0);
}
