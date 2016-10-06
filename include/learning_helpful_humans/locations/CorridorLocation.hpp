//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/AskLocation.hpp"

#ifndef PROJECT_CORRIDORLOCATION_HPP
#define PROJECT_CORRIDORLOCATION_HPP



class CorridorLocation : public AskLocation {
public:
    CorridorLocation();
    CorridorLocation(std::string name, std::string aspLocation);
    CorridorLocation(const CorridorLocation&);
    virtual ~CorridorLocation() { }
    virtual void goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&);

private:
};


#endif //PROJECT_CORRIDORLOCATION_HPP
