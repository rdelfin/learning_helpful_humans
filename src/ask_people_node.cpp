//
// Created by rdelfin on 10/3/16.
//

#include <ros/ros.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <learning_helpful_humans/locations/AskLocation.hpp>
#include <learning_helpful_humans/locations/LabLocation.hpp>
#include <learning_helpful_humans/locations/CorridorLocation.hpp>
#include <learning_helpful_humans/locations/OfficeLocation.hpp>

#include <fstream>
#include <string>
#include <vector>

#include <XmlRpcException.h>
#include <ros/package.h>

#include <json/json.hpp>

using json = nlohmann::json;

std::vector<AskLocation*> locations;

void loadLocations();

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ask_people_node");

    loadLocations();

    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> planClient("action_executor/execute_plan", true);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("move_base", true);
    planClient.waitForServer();
    moveBaseClient.waitForServer();


    int idx = 0;
    while(ros::ok()) {
        ROS_INFO_STREAM("Going to " << locations[idx]->getName()
                  << " (" << locations[idx]->getAspLocation()
                  << ") of type " << locations[idx]->getTypeString());

        locations[idx]->goToLocation(planClient, moveBaseClient);

        idx++;
        if(idx >= locations.size())
            idx = 0;
    }

    for(AskLocation* loc : locations)
        delete loc;

    return 0;
}

void loadLocations() {
    std::string locFilePath = ros::package::getPath("learning_helpful_humans") + "config/locations.json";
    ROS_INFO_STREAM("READING CONFIG FILE \"" << locFilePath << "\"");

    std::ifstream locFile(locFilePath, std::ifstream::in);
    std::string locString((std::istreambuf_iterator<char>(locFile)),
                           std::istreambuf_iterator<char>());

    // Load in JSON
    json locJson = json::parse(locString);

    json list = locJson["locations"];

    // Iterate over all locations
    for (json::iterator it = list.begin(); it != list.end(); ++it) {
        ROS_INFO("GETTING LOCATION");
        json item = *it;
        std::string type = item["type"];
        AskLocation* loc;
        if(type == "lab") {
            ROS_INFO("LAB");
            loc = new LabLocation();
            loc->load(item);
        } else if(type == "corridor") {
            ROS_INFO("CORRIDOR");
            loc = new CorridorLocation();
            loc->load(item);
        } else if(type == "office") {
            ROS_INFO("OFFICE");
            loc = new OfficeLocation();
            loc->load(item);
        } else
            continue;

        locations.push_back(loc);
    }

    ROS_INFO("DONE");
}
