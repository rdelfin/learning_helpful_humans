//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/OfficeLocation.hpp"

#include <string>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <bwi_msgs/Trigger.h>

#include <ros/ros.h>
#include <json/json.hpp>

using json = nlohmann::json;

OfficeLocation::OfficeLocation()
    : OfficeLocation("", "", "", "") {

}

OfficeLocation::OfficeLocation(std::string name, std::string aspLocation, std::string aspDoor, std::string aspCorridor)
    : AskLocation(name, aspLocation, LocationType::LOCATION_OFFICE), door(aspDoor), corridor(aspCorridor) {

}

OfficeLocation::OfficeLocation(const OfficeLocation& loc)
    : OfficeLocation(loc.name, loc.aspLocation, loc.door, loc.corridor) {

}

void OfficeLocation::load(XmlRpc::XmlRpcValue& val) {
    XmlRpc::XmlRpcValue door = val["door"];
    XmlRpc::XmlRpcValue name = val["name"];
    XmlRpc::XmlRpcValue location = val["location"];
    XmlRpc::XmlRpcValue corridor = val["corridor"];

    this->door = static_cast<std::string>(door);
    this->name = static_cast<std::string>(name);
    this->aspLocation = static_cast<std::string>(location);
    this->corridor = static_cast<std::string>(corridor);
}

void OfficeLocation::load(json& val) {
    this->door = val["door"];
    this->name = val["name"];
    this->aspLocation = val["location"];
    this->corridor = val["corridor"];
}

bool OfficeLocation::goOutsideLocation(actionlib::SimpleActionClient< bwi_kr_execution::ExecutePlanAction >& client,
                                       actionlib::SimpleActionClient< move_base_msgs::MoveBaseAction >&,
                                       ros::ServiceClient& stopClient, ros::ServiceClient& current_state_client) {
    bool success = goToCorridor(client, stopClient) && faceDoor(client, stopClient);
    return success;
}


bool OfficeLocation::goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client,
                                  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                                  ros::ServiceClient& stopClient, ros::ServiceClient& current_state_client) {
    bool success = goToCorridor(client, stopClient) &&
                   isDoorOpen(current_state_client) &&
                   faceDoor(client, stopClient) && enterRoom(client, stopClient) && faceDoor(client, stopClient);

    return success;
}

bool OfficeLocation::goToCorridor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client, ros::ServiceClient& stopClient) {
    // Go to corridor
    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not at";
    fluent.variables.push_back(corridor);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO_STREAM("Going to corridor (" << corridor << ")");

    client.sendGoal(goal);

    bool timed_out = !client.waitForResult(ros::Duration(200, 0));

    // If goal is not done in the timeout limit, cancel goal and return failed
    if (timed_out) {
        bwi_msgs::TriggerRequest req;
        bwi_msgs::TriggerResponse res;

        ROS_WARN_STREAM("Canceling goal to corridor: " << corridor);
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
        stopClient.call(req, res);
        return false;
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN_STREAM("Aborted goal to corridor: " << corridor);
        return false;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_WARN_STREAM("Preempted goal to corridor: " << corridor);
        return false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN_STREAM("Succeeded goal to corridor: " << corridor);
        return true;
    }
    else {
        ROS_WARN_STREAM("Terminated goal to corridor: " << corridor);
        return false;
    }
}

bool OfficeLocation::faceDoor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client, ros::ServiceClient& stopClient) {
    // Go to door
    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";
    fluent.variables.push_back(door);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO_STREAM("Facing door (" << door << ")");

    client.sendGoal(goal);

    client.waitForResult(ros::Duration(300, 0));

    // If goal is not done in the timeout limit (5s), cancel goal and return failed
    if (!client.getState().isDone()) {
        bwi_msgs::TriggerRequest req;
        bwi_msgs::TriggerResponse res;

        ROS_WARN_STREAM("Canceling goal to face door: " << door);
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
        stopClient.call(req, res);
        return false;
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN_STREAM("Aborted goal to face door: " << door);
        return false;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_WARN_STREAM("Preempted goal to face door: " << door);
        return false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN_STREAM("Succeeded goal to face door: " << door);
        return true;
    }
    else {
        ROS_WARN_STREAM("Terminated goal to face door: " << door);
        return false;
    }
}

bool OfficeLocation::isDoorOpen(ros::ServiceClient& client) {
    bwi_kr_execution::AspFluent openFluent;
    openFluent.name = "open";
    openFluent.timeStep = 0;
    openFluent.variables.push_back(door);

    ROS_INFO_STREAM("Checking if door (" << door << ") is open");

    bwi_kr_execution::AspRule rule;
    rule.head.push_back(openFluent);

    bwi_kr_execution::CurrentStateQuery csq;
    csq.request.query.push_back(rule);

    client.call(csq);

    return csq.response.answer.satisfied;
}

bool OfficeLocation::enterRoom(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client, ros::ServiceClient& stopClient) {
    // Goal to enter room
    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not at";
    fluent.variables.push_back(aspLocation);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO_STREAM("Entering room (" << aspLocation << ") is open");

    client.sendGoal(goal);

    client.waitForResult(ros::Duration(30, 0));

    // If goal is not done in the timeout limit (5s), cancel goal and return failed
    if (!client.getState().isDone()) {
        bwi_msgs::TriggerRequest req;
        bwi_msgs::TriggerResponse res;

        ROS_WARN_STREAM("Canceling goal to location: " << aspLocation);
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
        stopClient.call(req, res);
        return false;
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN_STREAM("Aborted goal to location " << aspLocation);
        return false;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_WARN_STREAM("Preempted goal to location " << aspLocation);
        return false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("Succeeded goal to location " << aspLocation);
        return true;
    }
    else {
        ROS_WARN_STREAM("Terminated goal to location " << aspLocation);
        return false;
    }
}
