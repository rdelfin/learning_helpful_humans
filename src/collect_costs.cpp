
/**
  * Created by Ricardo Delfin Garcia on 04/18/17.
  *
  * This node provides a set of services used to interact with the images on the firebase (online) database.
  * It provides `image_tool/upload` to upload new images and update old ones on firebase. It also provides
  * `image_tool/next` to fetch a random image using some internal heuristic. In the case that a UUID is
  * provided, it will fetch the image with said UUID. Finally, it provides  `image_tool/save_response` to
  * save an answer given by a user to an image.
  */

#include <ros/ros.h>
#include <signal.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <curlpp/cURLpp.hpp>

#include <actionlib/client/simple_action_client.h>
#include <learning_helpful_humans/locations/AskLocation.hpp>
#include <learning_helpful_humans/locations/LabLocation.hpp>
#include <learning_helpful_humans/locations/CorridorLocation.hpp>
#include <learning_helpful_humans/locations/OfficeLocation.hpp>
#include <learning_helpful_humans/request/GetFieldValue.h>
#include <learning_helpful_humans/request/TimeoutException.h>

#include <bwi_msgs/Trigger.h>

#include <unordered_map>

using json = nlohmann::json;

typedef std::pair<boost::uuids::uuid, boost::uuids::uuid> uuid_pair;

std::unordered_map<boost::uuids::uuid, AskLocation*, boost::hash<boost::uuids::uuid>> locationMap;
std::unordered_map<uuid_pair, int, boost::hash<uuid_pair>> locationVisitMap;

bool fetch_locations();

int main(int argc, char* argv[]) {
    // Setup cURL to accept multiple threads
    signal(SIGPIPE, SIG_IGN);
    
    ros::init(argc, argv, "collect_costs_node");
    ros::NodeHandle nh;
    
    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> planClient("/action_executor/execute_plan", true);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("/move_base", true);
    ros::ServiceClient stopClient = nh.serviceClient<bwi_msgs::Trigger>("stop_base");
    
    planClient.waitForServer();
    moveBaseClient.waitForServer();
    stopClient.waitForExistence();
    
    fetch_locations();
    
    ROS_INFO("Locations:");
    for(auto it = locationMap.begin(); it != locationMap.end(); ++it) {
        ROS_INFO_STREAM("\tLoc UUID: " << it->first);
        ROS_INFO_STREAM("\t\tName: " << it->second->getName());
        ROS_INFO_STREAM("\t\tASP Name: " << it->second->getAspLocation());
        ROS_INFO_STREAM("\t\tType: " << it->second->getTypeString());
    }
    
    ROS_INFO("Data:");
    int i = 0;
    for(auto it = locationVisitMap.begin(); it != locationVisitMap.end(); ++it) {
        i++;
        ROS_INFO_STREAM("\tLOCATION #" << i);
        ROS_INFO_STREAM("\t\tFrom: " << boost::uuids::to_string(it->first.first));
        ROS_INFO_STREAM("\t\tTo: " << boost::uuids::to_string(it->first.second));
        ROS_INFO_STREAM("\t\tTraveled " << it->second << " times");
    }
    
    return 0;
}

bool fetch_locations() {
    json locData, locTravelData;
    GetFieldValue getLocations("locations.json");
    GetFieldValue getLocationTravel("locationtravel.json");
    
    try {
        locData = getLocations.performAsJson();
        locTravelData = getLocationTravel.performAsJson();
    } catch(TimeoutException e) {
        ROS_ERROR_STREAM("The locations request timed out after " << e.time() << "seconds!");
        exit(-1);
    } catch(...) {
        ROS_ERROR_STREAM("The locations request failed for an unknown reason!");
        exit(-1);
    }
    
    for(auto it = locData.begin(); it != locData.end(); ++it) {
        json item = it.value();
        std::string uuid = it.key();
        std::string type = item["type"];
        AskLocation *loc;
        if (type == "lab") {
            loc = new LabLocation();
            loc->load(item);
        } else if (type == "corridor") {
            loc = new CorridorLocation();
            loc->load(item);
        } else if (type == "office") {
            loc = new OfficeLocation();
            loc->load(item);
        } else
            continue;

        locationMap[boost::lexical_cast<boost::uuids::uuid>(uuid)] = loc;
    }
    
    ROS_INFO("FETCHED ALL LOCATIONS");
    
    for(auto it_from = locationMap.begin(); it_from != locationMap.end(); ++it_from) {
        for (auto it_to = locationMap.begin(); it_to != locationMap.end(); ++it_to) {
            locationVisitMap[uuid_pair(it_from->first, it_to->first)] = 0;
        }
    }
    
    ROS_INFO("CREATED LOCATION VISIT MAP");
    
    ROS_INFO("LocTravelData:");
    ROS_INFO_STREAM("\t" << locTravelData);
    
    for(auto it = locTravelData.begin(); it != locTravelData.end(); ++it) {
        json obj = *it;
        std::string from = obj["from"];
        std::string to = obj["to"];
        uuid_pair key(boost::lexical_cast<boost::uuids::uuid>(from), boost::lexical_cast<boost::uuids::uuid>(to));
        if(locationVisitMap.count(key) == 0) {
            ROS_ERROR_STREAM("INVALID KEY FOUND:");
            ROS_ERROR_STREAM("\t(" << boost::uuids::to_string(key.first) << ", " << boost::uuids::to_string(key.second) << ")");
        } else {
            locationVisitMap[key]++;
        }
    }
    
    ROS_INFO("CREATED LOCATION VISIT MAP");
    
    /*for(auto it = locTravelData.begin(); it != locTravelData.end(); ++it) {
        json item = *it;
        uuid_pair pairIndex(boost::lexical_cast<boost::uuids::uuid>(item["from"]), boost::lexical_cast<boost::uuids::uuid>(item["to"]));
        locationVisitMap[pairIndex]++;
    }*/
}