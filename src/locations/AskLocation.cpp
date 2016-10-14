//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/AskLocation.hpp"

AskLocation::AskLocation() {

}

AskLocation::AskLocation(std::string name, std::string aspLocation, LocationType type)
    : name(name), aspLocation(aspLocation), type(type) {
}
AskLocation::AskLocation(const AskLocation& loc)
    : AskLocation(loc.name, loc.aspLocation, loc.type) {
}

const std::string& AskLocation::getName() { return name;  }
const std::string& AskLocation::getAspLocation() { return aspLocation; }
const std::string& AskLocation::getTypeString() {
    switch(type) {
        case LOCATION_LAB: return "lab";
        case LOCATION_CORRIDOR: return "corridor";
        case LOCATION_OFFICE: return "office";
        default: return "UNKNOWN LOCATION";
    }
}

