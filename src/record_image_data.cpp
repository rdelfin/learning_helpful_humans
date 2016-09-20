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

//openCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>  

#include "logging_utils.h"
#include "nav_utils.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

//works on v2 in both real world and simulation
#define IMAGE_TOPIC "/nav_kinect/rgb/image_raw"


#define IMG_DATA_PATH "/home/bwi/bwi_data"

bool heardImage = false;
sensor_msgs::Image current_image;

bool heardPose = false;
geometry_msgs::PoseWithCovarianceStamped current_pose;





void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	heardImage = true;
	current_image = *msg;
}

void poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
	heardPose = true;
	current_pose = *msg;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "between_doors");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  
  //create subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub_ = it.subscribe(IMAGE_TOPIC, 1, &imageCb);
  
  ros::Subscriber sub_pose = n.subscribe("/amcl_pose", 1, poseCb );
   
  std::string datapath(IMG_DATA_PATH);
  
  std::vector<string> doors;
  
  //lab doors
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  int current_door = 0;
    
    
    
    
    
  //office doors in pod
  //doors.push_back("d3_418");
  //doors.push_back("d3_432");
  
  
  //client for executing plans
  Client client("/action_executor/execute_plan", true);
  client.waitForServer();

  bool need_to_act = true;

  //spin rate
  double rate_hz = 10;
  ros::Rate r(rate_hz);

  double elapsed_time = 0;
  double save_every_k = 10; //save every k seconds
 
  double secs_prev =ros::Time::now().toSec();

  while (ros::ok()) {

	//if the robot is idle
    if (need_to_act){
		string location = doors.at(current_door);
		current_door++;
		if (current_door >= (int)doors.size())
			current_door = 0;

	   
		ROS_INFO_STREAM("going to " << location);

		bwi_kr_execution::ExecutePlanGoal goal = createDoorGoal(location);

		/*bwi_kr_execution::AspRule rule;
		bwi_kr_execution::AspFluent fluent;
		fluent.name = "not facing";

		fluent.variables.push_back(location);

		rule.body.push_back(fluent);
		goal.aspGoal.push_back(rule);*/

		ROS_INFO("sending goal");
		client.sendGoal(goal);
    
    
		need_to_act = false;
	}

    /*if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_INFO("Aborted");
    } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_INFO("Preempted");
    }*/

	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Succeeded!");
      need_to_act = true;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Aborted");
      need_to_act = true;
    } 


	ros::spinOnce();
	r.sleep();
	
	double secs_current =ros::Time::now().toSec();
	
	elapsed_time += (secs_current-secs_prev);
	secs_prev = secs_current;
	
	//save pose and image
	if (elapsed_time > save_every_k && heardImage && heardPose){
		ROS_INFO("Saving data...");
		
		elapsed_time -= save_every_k;
		
		ros::Time ts = ros::Time::now();
		
		long int ts_int = ts.toNSec();
		
		char filename_pose_c[90];
		sprintf(filename_pose_c,"%s/pose_%li.txt",datapath.c_str(),ts_int);
		std::string pose_filename(filename_pose_c);
		save_pose(pose_filename,current_pose.pose.pose);
		
		char filename_img_c[90];
		sprintf(filename_img_c,"%s/img_%li.png",datapath.c_str(),ts_int);
		std::string img_filename(filename_img_c);
		save_image(img_filename,current_image);
		
		
		//std::string pose_filename = IMG_DATA_PATH+"/"+ts_int+"_pose.txt";
	}
  }

  return 0;
}
