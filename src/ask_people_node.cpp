//
// Created by rdelfin on 10/3/16.
//

#include <ros/ros.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <string>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ask_people_node");

    std::vector<std::string> rooms;

    rooms.push_back("l3_414b");
    rooms.push_back("l3_400");
    rooms.push_back("l3_500");
    rooms.push_back("l3_432");

    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> client("action_executor/execute_plan", true);
    client.waitForServer();

    int idx = 0;
    while(ros::ok()) {
        bwi_kr_execution::ExecutePlanGoal goal;

        bwi_kr_execution::AspRule rule;
        bwi_kr_execution::AspFluent fluent;
        fluent.name = "not at";

        fluent.variables.push_back(rooms[idx]);

        rule.body.push_back(fluent);
        goal.aspGoal.push_back(rule);

        ROS_INFO_STREAM("Going to room " << rooms[idx]);
        client.sendGoal(goal);
        client.waitForResult(ros::Duration(120000, 0));

        if(!client.getState().isDone()) {
            ROS_INFO("Goal not done. Canceling and going to next room");
            client.cancelGoal();
        } else if(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal finished unsuccessfully");
        } else {
            ROS_INFO("Successfully arrived. Grabbing image and displaying...");
            // Do stuff
        }

        idx++;
        if(idx >= rooms.size())
            idx = 0;
    }

    return 0;
}