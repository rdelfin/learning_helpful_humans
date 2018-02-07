/**
  * Created by Ricardo Delfin Garcia on 10/3/16.
  * 
  * Provides useful services for fetching locations from config/locations.json. It loads in all the data
  * for the locations at startup and provides a different location every time through the
  * `next_question_location` service.
  */

#include <ros/ros.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <learning_helpful_humans/locations/AskLocation.hpp>
#include <learning_helpful_humans/locations/CorridorLocation.hpp>
#include <learning_helpful_humans/locations/LabLocation.hpp>
#include <learning_helpful_humans/locations/OfficeLocation.hpp>
#include <learning_helpful_humans/request/GetFieldValue.h>

#include <ros/package.h>

#include <json/json.hpp>
#include <bwi_msgs/ImageQuestion.h>
#include <bwi_msgs/NextLocation.h>
#include <bwi_msgs/Trigger.h>

#include <bwi_kr_execution/CurrentStateQuery.h>

using json = nlohmann::json;

std::vector<AskLocation*> locations;
actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>* planClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient;
ros::ServiceClient stopClient;
ros::ServiceClient currentStateClient;


int locIdx = 0;

bool nextQuestionCallback(bwi_msgs::NextLocationRequest& req, bwi_msgs::NextLocationResponse& res);

void loadLocations();

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "location_selector_node");
    ros::NodeHandle nh;

    loadLocations();

    ros::ServiceServer server = nh.advertiseService("next_question_location", nextQuestionCallback);
    planClient = new actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>("/action_executor/execute_plan", true);
    moveBaseClient = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true);
    stopClient = nh.serviceClient<bwi_msgs::Trigger>("stop_base");
    currentStateClient = nh.serviceClient<bwi_kr_execution::CurrentStateQuery> ("/current_state_query");

    planClient->waitForServer();
    moveBaseClient->waitForServer();
    stopClient.waitForExistence();

    ros::Rate r(10);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    for(AskLocation* loc : locations)
        delete loc;

    return 0;
}

void loadLocations() {
    try {
        GetFieldValue get_locations("locations.json");
        
        ROS_INFO("Requesting locations...");

        // Load in JSON
        json locJson = get_locations.performAsJson();

        // Iterate over all locations
        for (auto it = locJson.begin(); it != locJson.end(); ++it) {
            ROS_INFO("GETTING LOCATION");
            json val = it.value();
            val["id"] = it.key();
            
            std::string type = val["type"];
            AskLocation *loc;
            if (type == "lab") {
                ROS_INFO("LAB");
                loc = new LabLocation();
                loc->load(val);
            } else if (type == "corridor") {
                ROS_INFO("CORRIDOR");
                loc = new CorridorLocation();
                loc->load(val);
            } else if (type == "office") {
                ROS_INFO("OFFICE");
                loc = new OfficeLocation();
                loc->load(val);
            } else
                continue;

            locations.push_back(loc);
        }

        ROS_INFO("DONE");
    } catch(std::invalid_argument e) {
        std::cerr << "Error recieved! Invalid argument: " << e.what() << std::endl;
    }
}

bool nextQuestionCallback(bwi_msgs::NextLocationRequest& req, bwi_msgs::NextLocationResponse& res) {
    ROS_INFO_STREAM("Going to \"" << locations[locIdx]->getName()
              << "\" (" << locations[locIdx]->getAspLocation()
              << ") of type " << locations[locIdx]->getTypeString());

    res.locationName = locations[locIdx]->getName();
    res.success = locations[locIdx]->goToLocation(*planClient, *moveBaseClient, stopClient, currentStateClient);

    locIdx++;
    if(locIdx >= locations.size())
        locIdx = 0;
}
