//
// Created by rdelfin on 10/5/16.
//

#ifndef PROJECT_OFFICELOCATION_HPP
#define PROJECT_OFFICELOCATION_HPP

#include "AskLocation.hpp"

class OfficeLocation : public AskLocation {
public:
    OfficeLocation();
    OfficeLocation(std::string name, std::string aspLocation, std::string aspDoor);
    OfficeLocation(const CorridorLocation&);
    virtual ~OfficeLocation() { }
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&);
private:
    std::string door;

    bool goToDoor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client);
};

#endif //PROJECT_OFFICELOCATION_HPP
