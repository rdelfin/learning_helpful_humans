/*
 * The program which controls the robot while recording the data
 * 
 * 
 * 
 * 
 * 
 */ 
 
#include <stdlib.h>     /* srand, rand */

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>


#include "logging_utils.h"
#include "nav_utils.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;


bool go_to_target(struct TargetLocation T){
	Client client("/action_executor/execute_plan", true);
	client.waitForServer();
			
	if (T.type == "door"){
		
		bwi_kr_execution::ExecutePlanGoal goal = createDoorGoal(T.id);
		ROS_INFO("Sending goal:");
		ROS_INFO_STREAM(goal);
		
		client.sendGoalAndWait(goal);
		
		if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Succeeded!");
			return true;
		} 
		else if (client.getState() == actionlib::SimpleClientGoalState::ABORTED || client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
			ROS_INFO("Aborted or Pre-empted");
			return false;
		}
	}
	else if (T.type == "pose"){
		//first, make sure we're in the correct location area 
		bwi_kr_execution::ExecutePlanGoal goal = createLocationGoal(T.id);
		
		ROS_INFO("Sending goal:");
		ROS_INFO_STREAM(goal);
		
		client.sendGoalAndWait(goal);
		
		if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Succeeded!");
		} 
		else if (client.getState() == actionlib::SimpleClientGoalState::ABORTED || client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
			ROS_INFO("Aborted or Pre-empted");
			return false;
		}
		
		//next, send goal in /map frame of reference
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
		ac.waitForServer();
		
		move_base_msgs::MoveBaseGoal goal_mb;
		
		//set the header
		goal_mb.target_pose.header.stamp = ros::Time::now();
		goal_mb.target_pose.header.frame_id = "/map";
		
		//set relative x, y, and angle
		goal_mb.target_pose.pose = T.pose;

		//send the goal
		ac.sendGoal(goal_mb);
		
		//block until the action is completed
		ac.waitForResult();
		
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Succeeded!");
			return true;
		} 
		else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
			ROS_INFO("Aborted or Pre-empted");
			return false;
		}
	}
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "record_data_agent");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  
  //load locations to visit
  std::string pose_filename = ros::package::getPath("learning_helpful_humans")+"/data/pose_db.txt";
	
  std::vector<struct TargetLocation> L_set = getLocationsForExploration(pose_filename);
  
  for (unsigned int i = 0; i < L_set.size(); i++){
	  ROS_INFO("%s",L_set[i].id.c_str());
	  
	  if (L_set[i].type == "pose")
		ROS_INFO_STREAM(L_set[i].pose);
  }
  
  /*std::vector<std::string> doors;
  
  //lab doors
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  int current_door = 0;*/
    
    
  while (ros::ok()) {

	int random_location = rand() % L_set.size();

	ROS_INFO("Heading to location %i",random_location);

	bool result = go_to_target(L_set[random_location]);
	
	/*std::string location = doors.at(current_door);
	current_door++;
	if (current_door >= (int)doors.size())
		current_door = 0;

	   
	ROS_INFO_STREAM("going to " << location);

	bwi_kr_execution::ExecutePlanGoal goal = createDoorGoal(location);
	
	ROS_INFO("sending goal");
	client.sendGoalAndWait(goal);
    
    
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Succeeded!");
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_INFO("Aborted");
    } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_INFO("Preempted");
    }*/


	

  }

  return 0;
}
