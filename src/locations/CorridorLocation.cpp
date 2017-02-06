//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/CorridorLocation.hpp"

#include <bwi_kr_execution/ExecutePlanAction.h>

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <json/json.hpp>

using json = nlohmann::json;

CorridorLocation::CorridorLocation(std::string name, std::string aspLocation)
    : AskLocation(name, aspLocation, LocationType::LOCATION_CORRIDOR) {

}

CorridorLocation::CorridorLocation() : CorridorLocation("", "")  {

}

CorridorLocation::CorridorLocation(const CorridorLocation& cl)
    : AskLocation(cl.name, cl.aspLocation, cl.type) {

}

void CorridorLocation::load(XmlRpc::XmlRpcValue& val) {
    XmlRpc::XmlRpcValue name = val["name"];
    XmlRpc::XmlRpcValue location = val["location"];
    XmlRpc::XmlRpcValue pose = val["pose"];

    this->name = static_cast<std::string>(name);
    this->aspLocation = static_cast<std::string>(location);
    this->pose.position.x = static_cast<double>(pose["x"]);
    this->pose.position.y = static_cast<double>(pose["y"]);
    this->pose.position.z = 0.0f;
    this->pose.orientation = tf::createQuaternionMsgFromYaw(static_cast<double>(pose["theta"]));
}

void CorridorLocation::load(json& val) {
    json pose = val["pose"];

    this->name = val["name"];
    this->aspLocation = val["location"];
    this->pose.position.x = pose["x"];
    this->pose.position.y = pose["y"];
    this->pose.position.z = 0.0f;
    this->pose.orientation = tf::createQuaternionMsgFromYaw(static_cast<double>(pose["theta"]));
}

bool CorridorLocation::goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& planClient,
                                    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& poseClient) {
    return goToCorridor(planClient) && goToPose(poseClient);
}

bool CorridorLocation::goToCorridor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client) {
    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not at";
    fluent.variables.push_back(aspLocation);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO_STREAM("Going to " << aspLocation);
    client.sendGoal(goal);

    bool timed_out = client.waitForResult(ros::Duration(200, 0));

    // If goal is not done in the timeout limit, cancel goal and return failed
    if (timed_out) {
        ROS_WARN_STREAM("Canceling goal to location: " << aspLocation);
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
        ROS_WARN_STREAM("Succeeded goal to location " << aspLocation);
        return true;
    }
    else {
        ROS_WARN_STREAM("Terminated goal to location " << aspLocation);
        return false;
    }
}

bool CorridorLocation::goToPose(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& client) {
    ROS_INFO_STREAM("Going to pose (" << pose.position.x << ", " << pose.position.y << ", "
              << tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w).getAngle() << ")");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map";

    goal.target_pose.pose = pose;

    client.sendGoal(goal);

    client.waitForResult(ros::Duration(300, 0));

    // If goal is not done in the timeout limit (5s), cancel goal and return failed
    if (!client.getState().isDone()) {
        ROS_WARN_STREAM("Canceling goal to pose");
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
        return false;
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN_STREAM("Aborted goal to pose");
        return false;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_WARN_STREAM("Preempted goal to pose");
        return false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("Succeeded goal to pose");
        return true;
    }
    else {
        ROS_WARN_STREAM("Terminated goal to pose");
        return false;
    }
}

