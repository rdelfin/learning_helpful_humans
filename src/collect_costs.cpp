
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
#include <boost/graph/graph_concepts.hpp>

#include <curlpp/cURLpp.hpp>

#include <actionlib/client/simple_action_client.h>
#include <learning_helpful_humans/locations/AskLocation.hpp>
#include <learning_helpful_humans/locations/LabLocation.hpp>
#include <learning_helpful_humans/locations/CorridorLocation.hpp>
#include <learning_helpful_humans/locations/OfficeLocation.hpp>
#include <learning_helpful_humans/request/GetFieldValue.h>
#include <learning_helpful_humans/request/TimeoutException.h>
#include <learning_helpful_humans/request/AppendFieldValue.h>

#include <bwi_msgs/Trigger.h>

#include <unordered_map>

using json = nlohmann::json;

typedef std::pair<boost::uuids::uuid, boost::uuids::uuid> uuid_pair;

std::unordered_map<boost::uuids::uuid, AskLocation*, boost::hash<boost::uuids::uuid>> locationMap;
std::unordered_map<uuid_pair, int, boost::hash<uuid_pair>> locationVisitMap;

boost::uuids::uuid currentLocation;

bool locationKnown = false;

bool fetch_locations();
uuid_pair getNextLocation();
void sendTime(uuid_pair path, ros::Duration timeToTarget);

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
    
    while(ros::ok()) {
        uuid_pair path = getNextLocation();
        
        // Go to initial location
        if(!locationKnown)
            locationKnown = locationMap[path.first]->goToLocation(planClient, moveBaseClient, stopClient);
        
        if(locationKnown) {
            ros::Time start = ros::Time::now();
            locationMap[path.first]->goToLocation(planClient, moveBaseClient, stopClient);
            ros::Time end = ros::Time::now();
            
            ros::Duration timeToTarget = end - start;
            
            sendTime(path, timeToTarget);
        }
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
}

uuid_pair getNextLocation() {
    std::vector<uuid_pair> minElems;
    
    // First sweep: find minimum value
    int min = locationVisitMap.begin()->second;
    for(auto it = locationVisitMap.begin(); it != locationVisitMap.end(); ++it) {
        if(it->second < min)
            min = it->second;
    }
    
    for(auto it = locationVisitMap.begin(); it != locationVisitMap.end(); ++it)
        if(it->second == min)
            minElems.push_back(it->first);
        
    if(!locationKnown)
        return minElems[0];
    else {
        for(auto it = minElems.begin(); it != minElems.end(); ++it)
            if(it->first == currentLocation)
                return *it;
            
        return minElems[0];
    }
}

void sendTime(uuid_pair path, ros::Duration timeToTarget) {
    GetFieldValue locationTravelGet("locationtravel.json");
    json getData = locationTravelGet.performAsJson();
    
    json obj;
    obj["from"] = boost::uuids::to_string(path.first);
    obj["to"] = boost::uuids::to_string(path.second);
    obj["time"] = timeToTarget.toSec();
    obj["completed"] = true;
    
    json finalObj;
    finalObj[getData.size()] = obj;
    
    AppendFieldValue appendTime("locationtravel.json", finalObj);
    
    appendTime.perform();
}

