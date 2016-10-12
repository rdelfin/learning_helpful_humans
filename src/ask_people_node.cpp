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

#include <string>
#include <vector>

std::vector<AskLocation*> locations;

void loadLocations();

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ask_people_node");

    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> client("action_executor/execute_plan", true);
    client.waitForServer();

    loadLocations();

    int idx = 0;
    while(ros::ok()) {


        idx++;
        if(idx >= locations.size())
            idx = 0;
    }

    return 0;
}

void loadLocations() {
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue list;
    nh.getParam("/ask_image_location_data/locations", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(auto it = list.begin(); it != list.end(); ++it) {
        XmlRpc::XmlRpcValue item = it->second;
        XmlRpc::XmlRpcValue type = item["type"];
        ROS_ASSERT(type.getType() == XmlRpc::XmlRpcValue::TypeString);
        AskLocation* loc;
        if(type == "lab") {
            loc = new LabLocation();
            loc->load(item);
        } else if(type == "corridor") {
            loc = new CorridorLocation();
            loc->load(item);
        } else if(type == "office") {
            loc = new OfficeLocation();
            loc->load(item);
        } else
            continue;

        locations.push_back(loc);
    }
}