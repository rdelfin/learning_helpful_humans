#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>

#include <ros/ros.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

struct TargetLocation {
	
	std::string id; //e.g., "d3_414b" or "l3_200"
	
	std::string type; // "door", "location", "object", "pose"
	
	//only used for pose
	std::string location_name; //the location where the pose is if psoe
	geometry_msgs::Pose pose; //only used for "pose" types
};

struct TargetLocation createPoseLocation(geometry_msgs::Pose pose, std::string pose_id, std::string pose_loc){
	struct TargetLocation L;
	L.pose = pose;
	L.id = pose_id;
	L.type = std::string("pose");
	L.location_name = pose_loc;
	return L;
}

struct TargetLocation createLocationLocation(std::string location_name){
	struct TargetLocation L;
	L.id = location_name;
	L.type = std::string("location");
	return L;
}

struct TargetLocation createDoorLocation(std::string door_id){
	struct TargetLocation L;
	L.id = door_id;
	L.type = std::string("door");
	return L;
}

std::vector<struct TargetLocation> getLocationsForExploration(std::string pose_file){
	
	std::vector<struct TargetLocation> targets;
	
	//lab doors
	targets.push_back(createDoorLocation("d3_414b1"));
	targets.push_back(createDoorLocation("d3_414b2"));
	targets.push_back(createDoorLocation("d3_414a1"));
	targets.push_back(createDoorLocation("d3_414a2"));
	
	//offices
	targets.push_back(createDoorLocation("d3_432"));
	targets.push_back(createDoorLocation("d3_418"));
	
	//poses
	FILE *fp=fopen(pose_file.c_str(), "r");
	
	//the first integer is the # of entries in the file
	int num_entries = 0;
	fscanf(fp,"%i\n",&num_entries);
	
	for (int i = 0; i < num_entries; i++){
		char name_i[80];
		char location_i[80];
		float p_x,p_y,o_z,o_w;
			
		fscanf(fp,"%s\t%s\t%f,%f,%f,%f\n",name_i,location_i,&p_x,&p_y,&o_z,&o_w);
			
		geometry_msgs::Pose pose_i;
		pose_i.position.x = p_x;
		pose_i.position.y = p_y;
		pose_i.orientation.z = o_z;
		pose_i.orientation.w = o_w;
		
		std::string pose_i_name(name_i);
		std::string pose_i_loc(location_i);
		
		targets.push_back(createPoseLocation(pose_i,pose_i_name,pose_i_loc));
	}
	
	return targets;
}


bwi_kr_execution::ExecutePlanGoal createLocationGoal(std::string loc_id){
	bwi_kr_execution::ExecutePlanGoal goal;
  
	bwi_kr_execution::AspRule rule;
	bwi_kr_execution::AspFluent fluent;
	fluent.name = "not at";
  
	fluent.variables.push_back(loc_id);
 
	rule.body.push_back(fluent);
	goal.aspGoal.push_back(rule);
	
	return goal;
}

bwi_kr_execution::ExecutePlanGoal createDoorGoal(std::string door_id){
	bwi_kr_execution::ExecutePlanGoal goal;

	bwi_kr_execution::AspRule rule;
	bwi_kr_execution::AspFluent fluent;
	fluent.name = "not facing";

	fluent.variables.push_back(door_id);

	rule.body.push_back(fluent);
	goal.aspGoal.push_back(rule);
	
	return goal;
}
