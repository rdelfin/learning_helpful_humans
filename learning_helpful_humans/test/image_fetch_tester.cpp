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
#include <bwi_msgs/GetNextImage.h>

#include <pcl_conversions/pcl_conversions.h>

#include <vector>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_fetch_tester");
    ros::NodeHandle nh;
    
    DatabaseImage image;

    if(argc >= 2) {
        boost::uuids::uuid imageId = boost::lexical_cast<boost::uuids::uuid>(std::string(argv[1]));
        image = DatabaseImage(imageId);
        image.fetch();
    } else {
        ros::ServiceClient client = nh.serviceClient<bwi_msgs::GetNextImage>("/image_tool/next");
        ROS_INFO("Created client");
        bwi_msgs::GetNextImageRequest req;
        bwi_msgs::GetNextImageResponse res;

        client.call(req, res);
        ROS_INFO("Called client");
        
        
        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
        pcl::fromROSMsg(res.pc, pointCloud);
        
        
        image = DatabaseImage(res.img,
                              ImageMetadata(boost::lexical_cast<boost::uuids::uuid>(res.base_name), res.pose, std::vector<Answer>()),
                              pointCloud);
    }
    
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
