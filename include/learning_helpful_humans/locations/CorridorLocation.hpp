//
// Created by rdelfin on 10/5/16.
//

#include "learning_helpful_humans/locations/AskLocation.hpp"
#include <geometry_msgs/Pose.h>

#ifndef PROJECT_CORRIDORLOCATION_HPP
#define PROJECT_CORRIDORLOCATION_HPP



class CorridorLocation : public AskLocation {
public:
    CorridorLocation();
    CorridorLocation(std::string name, std::string aspLocation);
    CorridorLocation(const CorridorLocation&);
    virtual ~CorridorLocation() { }
    virtual bool goToLocation(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&,
                              actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&);

private:
    geometry_msgs::Pose pose;

    bool goToCorridor(actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>&);
    bool goToPose(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&);
};


#endif //PROJECT_CORRIDORLOCATION_HPP
