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

