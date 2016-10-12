//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/LabLocation.hpp"

#include <string>
#include <move_base_msgs/MoveBaseAction.h>

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

    this->door = door;
    this->name = name;
    this->aspLocation = location;
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

    client.sendGoal(goal);

    client.waitForResult(ros::Duration(300, 0));

    // If goal is not done in the timeout limit (5s), cancel goal and return failed
    if (!client.getState().isDone()) {
        ROS_ERROR_STREAM("Canceling goal to location: " << aspLocation);
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
        return false;
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_ERROR_STREAM("Aborted goal to location " << aspLocation);
        return false;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_ERROR_STREAM("Preempted goal to location " << aspLocation);
        return false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM("Succeeded goal to location " << aspLocation);
        return true;
    }
    else {
        ROS_ERROR_STREAM("Terminated goal to location " << aspLocation);
        return false;
    }
}
