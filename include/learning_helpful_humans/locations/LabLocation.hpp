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
    LabLocation(const CorridorLocation&);
    virtual ~LabLocation() { }
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&);
private:
    std::string door;
};

#endif //PROJECT_LABLOCATION_HPP
