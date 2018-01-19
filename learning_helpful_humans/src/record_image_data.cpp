/*
 * This node makes the robot traverse the environment while recording image 
 * data to be used in learning places experiment
 * 
 * 
 * 
 * 
 * 
 */ 


#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

//msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

//openCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include "logging_utils.h"
#include "nav_utils.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

//works on v2 in both real world and simulation
#define IMAGE_TOPIC "/nav_kinect/rgb/image_raw"
#define POINT_CLOUD_TOPIC "/nav_kinect/depth_registered/points"

#define IMG_DATA_PATH "/home/users/jsinapov/bwi_data"

bool heardImage = false;
sensor_msgs::Image current_image;

bool heardPose = false;
geometry_msgs::PoseWithCovarianceStamped current_pose;

bool heardCloud = false;
sensor_msgs::PointCloud2 current_cloud;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
PointCloudT::Ptr cloud (new PointCloudT);


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	heardImage = true;
	current_image = *msg;
}

void poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
	heardPose = true;
	current_pose = *msg;
}


void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	heardCloud = true;
	current_cloud = *input;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "record_image_data");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  
  //create subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub_ = it.subscribe(IMAGE_TOPIC, 1, &imageCb);
  
  ros::Subscriber sub_pose = n.subscribe("/amcl_pose", 1, poseCb );
  
  ros::Subscriber sub_pc = n.subscribe (POINT_CLOUD_TOPIC, 1, cloudCb);
 
   
   
  std::string datapath(IMG_DATA_PATH);
  
  std::vector<string> doors;
  
  
  
  //spin rate
  double rate_hz = 10;
  ros::Rate r(rate_hz);

  double elapsed_time = 0;
  double save_every_k = 30; //save every k seconds
 
  double secs_prev =ros::Time::now().toSec();

  char filename_pose_c[90];
  char filename_img_c[90];
  char filename_pc_c[90];


  std::vector< int > nan_filter;

  while (ros::ok()) {

	ros::spinOnce();
	
	//keep track of time
	double secs_current =ros::Time::now().toSec();
	elapsed_time += (secs_current-secs_prev);
	secs_prev = secs_current;
	
	//save pose and image
	if (elapsed_time > save_every_k && heardImage && heardPose){
		ROS_INFO("Saving data...");
		
		elapsed_time -= save_every_k;
		
		ros::Time ts = ros::Time::now();
		
		long int ts_int = ts.toNSec();
		
		
		sprintf(filename_pose_c,"%s/pose_%li.txt",datapath.c_str(),ts_int);
		std::string pose_filename(filename_pose_c);
		save_pose(pose_filename,current_pose.pose.pose);
		
		sprintf(filename_img_c,"%s/img_%li.png",datapath.c_str(),ts_int);
		std::string img_filename(filename_img_c);
		save_image(img_filename,current_image);
		
		//convert point cloud
		//convert to PCL format
		pcl::fromROSMsg (current_cloud, *cloud);
		pcl::removeNaNFromPointCloud(*cloud,*cloud,nan_filter);
		sprintf(filename_pc_c,"%s/cloud_%li.pcd",datapath.c_str(),ts_int);

		pcl::io::savePCDFileASCII (filename_pc_c, *cloud);
		
		//std::string pose_filename = IMG_DATA_PATH+"/"+ts_int+"_pose.txt";
	}
	
	r.sleep();
  }

  return 0;
}
