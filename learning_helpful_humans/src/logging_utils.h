#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>

//openCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

#include <geometry_msgs/Pose.h>

void save_pose(std::string filename, geometry_msgs::Pose p){
	FILE *fp_c=fopen(filename.c_str(), "w");
	
	fprintf(fp_c,"%f,%f,%f,%f,%f,%f,%f",p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
	
	fclose(fp_c);
}

void save_image(std::string filename, sensor_msgs::Image img){
	
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat original_image = cv_ptr->image;
    
    imwrite( filename, original_image);
    
}
