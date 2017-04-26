//
// Created by rdelfin on 10/5/16.
//

#ifndef PROJECT_ASKLOCATION_HPP
#define PROJECT_ASKLOCATION_HPP

#include <move_base_msgs/MoveBaseAction.h>

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <bwi_kr_execution/ExecutePlanAction.h>

#include <json/json.hpp>

enum LocationType {
    LOCATION_NONE,     // Used for default. No child should use this
    LOCATION_CORRIDOR,
    LOCATION_LAB,
    LOCATION_OFFICE
};

class AskLocation {
public:
    AskLocation();
    AskLocation(std::string name, std::string aspLocation, LocationType type);
    AskLocation(const AskLocation&);
    virtual ~AskLocation() { }
    virtual void load(XmlRpc::XmlRpcValue&) = 0;
    virtual void load(nlohmann::json&) = 0;
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                              actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                              ros::ServiceClient&) = 0;
    virtual bool goOutsideLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                                   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                                   ros::ServiceClient&) = 0;

    const std::string& getName();
    const std::string& getAspLocation();
    std::string getTypeString();

protected:
    std::string name;
    std::string aspLocation;
    LocationType type;
};



#endif //PROJECT_ASKLOCATION_HPP
