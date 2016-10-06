//
// Created by rdelfin on 10/5/16.
//

#ifndef PROJECT_ASKLOCATION_HPP
#define PROJECT_ASKLOCATION_HPP

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <bwi_kr_execution/ExecutePlanAction.h>

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
    virtual void goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&) = 0;
protected:
    std::string name;
    std::string aspLocation;
    LocationType type;
};



#endif //PROJECT_ASKLOCATION_HPP
