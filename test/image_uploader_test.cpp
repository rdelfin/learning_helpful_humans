#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bwi_msgs/UploadImage.h>
#include <cv_bridge/cv_bridge.h>

sensor_msgs::PointCloud2 loadPointCloud(std::string filePath);
sensor_msgs::Image loadImage(std::string filePath);
geometry_msgs::Pose loadPose(std::string filePath);
std::vector<std::string> split(std::string str, char delimiter);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_uploader_test");
    ros::NodeHandle nh;

    if(argc != 4) {
        std::cerr << "two arguments required for image_uploader_test" << std::endl;
        return -1;
    }

    ros::ServiceClient client = nh.serviceClient<bwi_msgs::UploadImage>("image_tool/upload");
    client.waitForExistence();
    if(!client.exists()) {
        std::cerr << "The image upload service server does not exist. Start up node before running this script" << std::endl;
        return -2;
    }

    bwi_msgs::UploadImageRequest req;
    bwi_msgs::UploadImageResponse res;

    std::string imagePath(argv[1]);
    std::string pcPath(argv[2]);
    std::string posePath(argv[3]);

    req.pose = loadPose(posePath);
    req.pc = loadPointCloud(pcPath);
    req.img = loadImage(imagePath);

    client.call(req, res);

    std::cerr << "Response: {success: " << (res.success ? "true" : "false") << "}" << std::endl;

    return 0;
}

sensor_msgs::PointCloud2 loadPointCloud(std::string filePath) {
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pcl::io::loadPCDFile(filePath, pointCloud);

    sensor_msgs::PointCloud2 rosPc;
    pcl::PCLPointCloud2 pc2;
    pcl::toPCLPointCloud2(pointCloud, pc2);
    pcl_conversions::fromPCL(pc2, rosPc);

    return rosPc;
}

sensor_msgs::Image loadImage(std::string filePath) {
    cv::Mat imageMat = cv::imread(filePath);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    return *cv_bridge::CvImage(header, "bgr8", imageMat).toImageMsg();
}

geometry_msgs::Pose loadPose(std::string filePath) {
    std::ifstream poseFile(filePath);
    std::string poseString((std::istreambuf_iterator<char>(poseFile)),   // Read file into string
                           std::istreambuf_iterator<char>());

    // Parse into a pose object
    std::vector<std::string> csvList = split(poseString, ',');
    geometry_msgs::Pose pose;
    pose.position.x = std::stod(csvList[0]);
    pose.position.y = std::stod(csvList[1]);
    pose.position.z = std::stod(csvList[2]);
    pose.orientation.x = std::stod(csvList[3]);
    pose.orientation.y = std::stod(csvList[4]);
    pose.orientation.z = std::stod(csvList[5]);
    pose.orientation.w = std::stod(csvList[6]);

    return pose;
}

// Obtained from http://code.runnable.com/VHb0hWMZp-ws1gAr/splitting-a-string-into-a-vector-for-c%2B%2B
std::vector<std::string> split(std::string str, char delimiter) {
    std::vector<std::string> internal;
    std::stringstream ss(str); // Turn the string into a stream.
    std::string tok;

    while(std::getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }

    return internal;
}