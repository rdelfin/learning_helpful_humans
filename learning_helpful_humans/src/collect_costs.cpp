
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

#include <bwi_kr_execution/CurrentStateQuery.h>

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
    
    ros::ServiceClient currentStateClient = nh.serviceClient<bwi_kr_execution::CurrentStateQuery> ("/current_state_query");
    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> planClient("/action_executor/execute_plan", true);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("/move_base", true);
    ros::ServiceClient stopClient = nh.serviceClient<bwi_msgs::Trigger>("stop_base");
    
    //planClient.waitForServer();
    //moveBaseClient.waitForServer();
    //stopClient.waitForExistence();
    //currentStateClient.waitForExistence();
    
    fetch_locations();
    
    while(ros::ok()) {
        uuid_pair path = getNextLocation();
        ROS_INFO_STREAM("Going from location ID: {" << boost::uuids::to_string(path.first) << "}  to {" << boost::uuids::to_string(path.second) << "}");
        ROS_INFO_STREAM("\tFirst location exists?:    " << (locationMap.count(path.first) > 0 ? "yes" : "no"));
        ROS_INFO_STREAM("\tSecond location exists?:   " << (locationMap.count(path.second) > 0 ? "yes" : "no"));
        ROS_INFO_STREAM("\tPair in visit map exists?: " << (locationVisitMap.count(path) > 0 ? "yes" : "no"));
        
        // Go to initial location
        if(!locationKnown) {
            ROS_INFO_STREAM("Going to initial location... (" << locationMap[path.first]->getName() << ")");
            locationKnown = locationMap[path.first]->goOutsideLocation(planClient, moveBaseClient, stopClient, currentStateClient);
            ROS_INFO_STREAM("Finished going to initial location " << locationMap[path.first]->getName() << "). Success: " << (locationKnown ? "true" : "false"));
        }
        
        if(locationKnown) {
            ros::Time start = ros::Time::now();
            ROS_INFO_STREAM("Going to second location... (" << locationMap[path.second]->getName() << ")");
            bool success = locationMap[path.second]->goOutsideLocation(planClient, moveBaseClient, stopClient, currentStateClient);
            ros::Time end = ros::Time::now();
            
            ros::Duration timeToTarget = end - start;
            ROS_INFO_STREAM("Arrived " << (success ? "" : "un") << "successfully to location " << locationMap[path.second]->getName() << " after " << timeToTarget.toSec() <<" seconds.");
            
            sendTime(path, timeToTarget);
        }

        locationVisitMap[path]++;
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
        return false;
    } catch(...) {
        ROS_ERROR_STREAM("The locations request failed for an unknown reason!");
        return false;
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
            ROS_INFO_STREAM("Load corridor location: " << item);
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
    std::vector<uuid_pair> min_list;
    std::pair<uuid_pair, int> test = *locationVisitMap.begin();

    auto min_comp = [](const std::pair<uuid_pair, int>& a, const std::pair<uuid_pair, int>& b) -> bool {
        return a.second < b.second;
    };


    int min_val = std::min_element(locationVisitMap.begin(), locationVisitMap.end(), min_comp)->second;

    auto val_comp = [&](const std::pair<uuid_pair, int> a) -> bool {
        return a.second == min_val;
    };

    {
        auto it = locationVisitMap.begin();
        while (it != locationVisitMap.end()) {
            it = std::find_if(it, locationVisitMap.end(), val_comp);
            if (it != locationVisitMap.end()) {
                min_list.push_back(it->first);
                ++it;
            }
        }
    }

    if(!locationKnown)
        return min_list[0];
    else {
        for(auto it = min_list.begin(); it != min_list.end(); ++it)
            if(it->first == currentLocation)
                return *it;

        return min_list[0];
    }
}

void sendTime(uuid_pair path, ros::Duration timeToTarget) {
    GetFieldValue locationTravelGet("locationtravel.json");
    ROS_INFO("Preforming location travel get...");
    json getData = locationTravelGet.performAsJson();
    ROS_INFO("Location travel get succeeded!");
    
    json obj;
    obj["from"] = boost::uuids::to_string(path.first);
    obj["to"] = boost::uuids::to_string(path.second);
    obj["time"] = timeToTarget.toSec();
    obj["completed"] = true;
    
    json finalObj = json::object();
    finalObj[std::to_string(getData.size())] = obj;

    ROS_INFO_STREAM("Sending JSON object to /locationtravel.json: " << finalObj);
    AppendFieldValue appendTime("locationtravel.json", finalObj);
    
    appendTime.perform();
    ROS_INFO_STREAM("Object sent to /loctaiontravel.json!");
}

