/**
  * Created by Ricardo Delfin Garcia on 10/3/16.
  * 
  * Provides useful services for fetching locations from config/locations.json. It loads in all the data
  * for the locations at startup and provides a different location every time through the
  * `next_question_location` service.
  */

#include <ros/ros.h>

#include <bwi_kr_execution/ExecutePlanAction.h>
#include <bwi_kr_execution/CurrentStateQuery.h>

#include <bwi_msgs/ActionUpdate.h>
#include <bwi_msgs/ImageQuestion.h>
#include <bwi_msgs/LocationAction.h>
#include <bwi_msgs/NextLocation.h>
#include <bwi_msgs/Trigger.h>

#include <actionlib/client/simple_action_client.h>

#include <learning_helpful_humans/locations/AskLocation.hpp>
#include <learning_helpful_humans/locations/CorridorLocation.hpp>
#include <learning_helpful_humans/locations/LabLocation.hpp>
#include <learning_helpful_humans/locations/OfficeLocation.hpp>
#include <learning_helpful_humans/request/GetFieldValue.h>

#include <json/json.hpp>

#include <ctime>
#include <random>
#include <unordered_map>


using json = nlohmann::json;

std::unordered_map<std::string, AskLocation*> locations;
actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>* planClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient;
ros::ServiceClient stopClient;
ros::ServiceClient currentStateClient;
ros::ServiceClient getSarsaActionClient;

bool nextQuestionCallback(bwi_msgs::NextLocationRequest& req, bwi_msgs::NextLocationResponse& res);

void loadLocations();

bool location_known = false;
std::string current_location_id = "";
std::default_random_engine generator;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "location_selector_node");
    ros::NodeHandle nh;

    loadLocations();

    ros::ServiceServer server = nh.advertiseService("next_question_location", nextQuestionCallback);
    planClient = new actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction>("/action_executor/execute_plan", true);
    moveBaseClient = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true);
    stopClient = nh.serviceClient<bwi_msgs::Trigger>("stop_base");
    currentStateClient = nh.serviceClient<bwi_kr_execution::CurrentStateQuery> ("/current_state_query");
    getSarsaActionClient = nh.serviceClient<bwi_msgs::LocationAction>("next_sarsa_action");

    planClient->waitForServer();
    moveBaseClient->waitForServer();
    stopClient.waitForExistence();
    currentStateClient.waitForExistence();
    getSarsaActionClient.waitForExistence();

    ros::Rate r(10);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    for(std::pair<std::string, AskLocation*> loc : locations)
        delete loc.second;

    return 0;
}

void loadLocations() {
    try {
        GetFieldValue get_locations("locations.json");
        
        ROS_INFO("Requesting locations...");

        // Load in JSON
        json locJson = get_locations.performAsJson();

        // Iterate over all locations
        for (auto it = locJson.begin(); it != locJson.end(); ++it) {
            ROS_INFO("GETTING LOCATION");
            json val = it.value();
            val["id"] = it.key();
            
            std::string type = val["type"];
            AskLocation *loc;
            if (type == "lab") {
                ROS_INFO("LAB");
                loc = new LabLocation();
                loc->load(val);
            } else if (type == "corridor") {
                ROS_INFO("CORRIDOR");
                loc = new CorridorLocation();
                loc->load(val);
            } else if (type == "office") {
                ROS_INFO("OFFICE");
                loc = new OfficeLocation();
                loc->load(val);
            } else
                continue;

            locations[it.key()] = loc;
        }

        ROS_INFO("DONE");
    } catch(std::invalid_argument e) {
        std::cerr << "Error recieved! Invalid argument: " << e.what() << std::endl;
    }
}

int day_time_seconds() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_sec + 60*(now->tm_min + 60*now->tm_hour);
}

int day_week() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_wday == 0 ? 6 : now->tm_wday - 1;
}

std::pair<std::string, bool> get_next_action() {
    bwi_msgs::LocationActionRequest actionRequest;
    bwi_msgs::LocationActionResponse actionResponse;

    actionRequest.current_location = current_location_id;
    actionRequest.time.time_seconds = day_time_seconds();
    actionRequest.time.day_of_week = day_week();

    getSarsaActionClient.call(actionRequest, actionResponse);

    return {actionResponse.location, actionResponse.ask_question};
}

bool nextQuestionCallback(bwi_msgs::NextLocationRequest& req, bwi_msgs::NextLocationResponse& res) {

    // If the location isn't known already, go to a random location and start from there.
    if(!location_known) {
        std::uniform_int_distribution<int> dist(0, locations.size() - 1);
        auto item = locations.begin();
        std::advance(item, dist(generator));
        std::string next_id = item->first;

        locations[next_id]->goToLocation(*planClient, *moveBaseClient, stopClient, currentStateClient);
        current_location_id = next_id;
        location_known = true;
    }

    std::pair<std::string, bool> next_action = get_next_action();

    AskLocation* loc = locations[next_action.first];
    ROS_INFO_STREAM("Going to \"" << locations[next_action.first]->getName()
              << "\" (" << locations[next_action.first]->getAspLocation()
              << ") of type " << locations[next_action.first]->getTypeString());

    res.locationName = locations[next_action.first]->getName();
    res.success = locations[next_action.first]->goToLocation(*planClient, *moveBaseClient, stopClient, currentStateClient);
}
