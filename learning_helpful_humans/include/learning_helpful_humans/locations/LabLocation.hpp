//
// Created by rdelfin on 10/5/16.
//

#ifndef PROJECT_LABLOCATION_HPP
#define PROJECT_LABLOCATION_HPP

#include "AskLocation.hpp"

class LabLocation : public AskLocation {
public:
    LabLocation();
    LabLocation(std::string name, std::string aspLocation, std::string aspDoor);
    LabLocation(const LabLocation&);
    virtual ~LabLocation() { }
    virtual void load(XmlRpc::XmlRpcValue&);
    virtual void load(nlohmann::json&);
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                              actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                              ros::ServiceClient&, ros::ServiceClient& current_state_client);
    virtual bool goOutsideLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                                   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&,
                                   ros::ServiceClient&, ros::ServiceClient& current_state_client);
private:
    std::string door;
};

#endif //PROJECT_LABLOCATION_HPP
