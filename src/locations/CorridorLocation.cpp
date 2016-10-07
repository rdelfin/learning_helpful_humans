//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/CorridorLocation.hpp"

#include <bwi_kr_execution/ExecutePlanAction.h>

#include <string>

#include <actionlib/client/simple_action_client.h>

CorridorLocation::CorridorLocation(std::string name, std::string aspLocation)
    : AskLocation(name, aspLocation, LocationType::LOCATION_CORRIDOR) {

}

CorridorLocation::CorridorLocation() : CorridorLocation("", "")  {

}

CorridorLocation::CorridorLocation(const CorridorLocation& cl)
    : AskLocation(cl.name, cl.name, cl.type) {

}

bool CorridorLocation::goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>& client) {

    client.sendGoal();
}
