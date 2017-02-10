//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/LabLocation.hpp"

#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <json/json.hpp>

using json = nlohmann::json;

LabLocation::LabLocation(std::string name, std::string aspLocation, std::string aspDoor)
        : AskLocation(name, aspLocation, LocationType::LOCATION_LAB), door(aspDoor) {

}

LabLocation::LabLocation() : LabLocation("", "", "")  {

}

LabLocation::LabLocation(const LabLocation& cl)
        : LabLocation(cl.name, cl.aspLocation, cl.door) {

}

void LabLocation::load(XmlRpc::XmlRpcValue& val) {
    XmlRpc::XmlRpcValue door = val["door"];
    XmlRpc::XmlRpcValue name = val["name"];
    XmlRpc::XmlRpcValue location = val["location"];

    this->door = static_cast<std::string>(door);
    this->name = static_cast<std::string>(name);
    this->aspLocation = static_cast<std::string>(location);
}

void LabLocation::load(json& val) {
    this->door = val["door"];
    this->name = val["name"];
    this->aspLocation = val["location"];
}

bool LabLocation::goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client,
                               actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&) {
    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not at";
    fluent.variables.push_back(aspLocation);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO_STREAM("Going to location " << aspLocation);

    client.sendGoal(goal);

    bool timed_out = !client.waitForResult(ros::Duration(200, 0));

    // If goal is not done in the timeout limit, cancel goal and return failed
    if (timed_out) {
        // TODO: Send stop command
        ROS_WARN_STREAM("Canceling goal to location: " << aspLocation);
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
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
