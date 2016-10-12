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
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                              actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&);
private:
    std::string door;
};

#endif //PROJECT_LABLOCATION_HPP
