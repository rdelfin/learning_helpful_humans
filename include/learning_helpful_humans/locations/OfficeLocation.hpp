//
// Created by rdelfin on 10/5/16.
//

#ifndef PROJECT_OFFICELOCATION_HPP
#define PROJECT_OFFICELOCATION_HPP

#include "AskLocation.hpp"

class OfficeLocation : public AskLocation {
public:
    OfficeLocation();
    OfficeLocation(std::string name, std::string aspLocation, std::string aspDoor, std::string aspCorridor);
    OfficeLocation(const OfficeLocation&);
    virtual ~OfficeLocation() { }
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                              actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&);
private:
    std::string door;
    std::string corridor;

    bool faceDoor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client);
    // Necessary because asp goToDoor action fails when it has to cross another door to get to said door
    bool goToCorridor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client);

    bool isDoorOpen(ros::ServiceClient& client);

    bool enterRoom(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client);
};

#endif //PROJECT_OFFICELOCATION_HPP
