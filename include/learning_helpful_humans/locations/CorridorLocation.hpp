//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/AskLocation.hpp"
#include <geometry_msgs/Pose.h>

#ifndef PROJECT_CORRIDORLOCATION_HPP
#define PROJECT_CORRIDORLOCATION_HPP



class CorridorLocation : public AskLocation {
public:
    CorridorLocation();
    CorridorLocation(std::string name, std::string aspLocation);
    CorridorLocation(const CorridorLocation&);
    virtual ~CorridorLocation() { }
    virtual void load(XmlRpc::XmlRpcValue&);
    virtual void load(nlohmann::json&);
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                              actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                              ros::ServiceClient&, ros::ServiceClient& current_state_client);
    virtual bool goOutsideLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                                   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                                   ros::ServiceClient&, ros::ServiceClient& current_state_client);

private:
    geometry_msgs::Pose pose;

    bool goToCorridor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&, ros::ServiceClient&);
    bool goToPose(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&, ros::ServiceClient&);
};


#endif //PROJECT_CORRIDORLOCATION_HPP
