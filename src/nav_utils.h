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



struct TargetLocation createLocationLocation(std::string location_name){
	struct TargetLocation L;
	L.id = location_name;
	L.type = std::string("location");
}

struct TargetLocation createDoorLocation(std::string door_id){
	struct TargetLocation L;
	L.id = door_id;
	L.type = std::string("door");
}

std::vector<struct TargetLocation> getLocationsForExploration(){
	
	std::vector<struct TargetLocation> targets;
	
	//lab doors
	targets.push_back(createDoorLocation("d3_414b1"));
	targets.push_back(createDoorLocation("d3_414b2"));
	targets.push_back(createDoorLocation("d3_414a1"));
	targets.push_back(createDoorLocation("d3_414a2"));
	
	//offices
	targets.push_back(createDoorLocation("d3_432"));
	targets.push_back(createDoorLocation("d3_418"));
	
	
	return targets;
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
