/**
  * Created by rdelfin on 10/3/16.
  * Provides useful services for fetching locations from config/locations.json. It loads in all the data
  * for the locations at startup and provides a different location every time through the
  * `next_question_location` service.
  */

#include <ros/ros.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <learning_helpful_humans/locations/AskLocation.hpp>
#include <learning_helpful_humans/locations/LabLocation.hpp>
#include <learning_helpful_humans/locations/CorridorLocation.hpp>
#include <learning_helpful_humans/locations/OfficeLocation.hpp>

#include <fstream>

#include <ros/package.h>

#include <json/json.hpp>
#include <bwi_msgs/ImageQuestion.h>
#include <bwi_msgs/NextLocation.h>

using json = nlohmann::json;

std::vector<AskLocation*> locations;
actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>* planClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient;


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
    planClient->waitForServer();
    moveBaseClient->waitForServer();

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
        std::string locFilePath = ros::package::getPath("learning_helpful_humans") + "/config/locations.json";
        ROS_INFO_STREAM("READING CONFIG FILE \"" << locFilePath << "\"");

        std::ifstream locFile(locFilePath, std::ifstream::in);
        std::string locString((std::istreambuf_iterator<char>(locFile)),
                              std::istreambuf_iterator<char>());

        ROS_INFO("Parsing JSON...");

        // Load in JSON
        json locJson = json::parse(locString);


        json list = locJson["locations"];

        // Iterate over all locations
        for (json::iterator it = list.begin(); it != list.end(); ++it) {
            ROS_INFO("GETTING LOCATION");
            json item = *it;
            std::string type = item["type"];
            AskLocation *loc;
            if (type == "lab") {
                ROS_INFO("LAB");
                loc = new LabLocation();
                loc->load(item);
            } else if (type == "corridor") {
                ROS_INFO("CORRIDOR");
                loc = new CorridorLocation();
                loc->load(item);
            } else if (type == "office") {
                ROS_INFO("OFFICE");
                loc = new OfficeLocation();
                loc->load(item);
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
    res.success = locations[locIdx]->goToLocation(*planClient, *moveBaseClient);

    locIdx++;
    if(locIdx >= locations.size())
        locIdx = 0;
}
