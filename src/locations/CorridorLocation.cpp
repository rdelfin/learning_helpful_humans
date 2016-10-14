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
    this->name = name;
    this->aspLocation = location;
    this->pose.position.x = pose["x"];
    this->pose.position.y = pose["y"];
    this->pose.position.z = 0.0f;
    this->pose.orientation = tf::createQuaternionMsgFromYaw(pose["theta"]);
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
        ROS_ERROR_STREAM("Canceling goal to pose");
        client.cancelGoal();
        client.waitForResult(ros::Duration(1, 0));
        return false;
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_ERROR_STREAM("Aborted goal to pose");
        return false;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_ERROR_STREAM("Preempted goal to pose");
        return false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR_STREAM("Succeeded goal to pose");
        return true;
    }
    else {
        ROS_ERROR_STREAM("Terminated goal to pose");
        return false;
    }
}

