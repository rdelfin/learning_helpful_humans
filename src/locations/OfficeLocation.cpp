//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/OfficeLocation.hpp"

#include <string>

OfficeLocation::OfficeLocation()
    : OfficeLocation("", "", "") {

}

OfficeLocation::OfficeLocation(std::string name, std::string aspLocation, std::string aspDoor)
    : AskLocation(name, aspLocation, LocationType::LOCATION_OFFICE) {

}

OfficeLocation::OfficeLocation(const OfficeLocation& loc)
    : OfficeLocation(loc.name, loc.aspLocation, loc.door) {

}

bool OfficeLocation::goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client) {
    bool goToDoorSuccess = goToDoor(client);
}

bool OfficeLocation::goToDoor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client) {
    // Go to door
    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";
    fluent.variables.push_back(door);
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
