/**
  * Created by Ricardo Delfin Garcia on 10/18/16.
  *
  * This drives the entire asking mechanism. Contains the overall logic: grab a location, go to it,
  * trigger the question asking, and save it onto the database. Uses all other nodes for all this
  * task, so it does very little work by itself.
  */

#include <bwi_msgs/ActionUpdate.h>
#include <bwi_msgs/GetNextImage.h>
#include <bwi_msgs/ImageQuestion.h>
#include <bwi_msgs/NextLocation.h>
#include <bwi_msgs/SaveImageResponse.h>
#include <bwi_msgs/Trigger.h>

#include <cv_bridge/cv_bridge.h>

#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <learning_helpful_humans/Question.h>

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>

#include <sound_play/sound_play.h>

#include <opencv2/opencv.hpp>

bwi_msgs::GetNextImageResponse getQuestionImg();

ros::ServiceClient nextImageClient;
ros::ServiceClient saveResponseClient;
ros::ServiceClient updateSarsaClient;

int day_time_seconds();
int day_week();

void update_sarsa_agent(std::string location, int result);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "location_asker_node");

    ros::NodeHandle nh;

    ros::ServiceClient askerClient = nh.serviceClient<bwi_msgs::ImageQuestion>("ask_location");
    ros::ServiceClient locationClient = nh.serviceClient<bwi_msgs::NextLocation>("next_question_location");
    nextImageClient = nh.serviceClient<bwi_msgs::GetNextImage>("image_tool/next");
    saveResponseClient = nh.serviceClient<bwi_msgs::SaveImageResponse>("image_tool/save_response");
    updateSarsaClient = nh.serviceClient<bwi_msgs::ActionUpdate>("update_sarsa_action");

    ros::Publisher pub = nh.advertise<sound_play::SoundRequest>("robotsound", 10);

    // Ensure the node does not start until all other node dependencies are up.
    askerClient.waitForExistence();
    locationClient.waitForExistence();
    nextImageClient.waitForExistence();
    saveResponseClient.waitForExistence();
    updateSarsaClient.waitForExistence();

    ros::Rate r(10);

    bwi_msgs::NextLocationRequest locReq;
    bwi_msgs::NextLocationResponse locRes;
    bwi_msgs::ImageQuestionRequest qReq;
    bwi_msgs::ImageQuestionResponse qRes;
    sound_play::SoundRequest helpSound;
    sound_play::SoundRequest thanksSound;

    bwi_msgs::ActionUpdateRequest actionUpdateReq;

    helpSound.sound = thanksSound.sound = sound_play::SoundRequest::SAY;
    helpSound.command = thanksSound.command = sound_play::SoundRequest::PLAY_ONCE;
    helpSound.arg = "Hello! Can you help me by answering a quick question?";
    thanksSound.arg = "Thank you!";

    while(ros::ok()) {
        ros::spinOnce();

        locationClient.call(locReq, locRes);

        if(locRes.success) {
            ROS_WARN_STREAM("FAILED TO GO TO NEXT LOCATION.");
            update_sarsa_agent(locRes.locationName, actionUpdateReq.ARRIVAL_FAILURE);
            continue;
        }
        else
            ROS_INFO_STREAM("SUCCESSFULLY ARRIVED TO NEXT LOCATION.");

        pub.publish(helpSound);

        // Fetch a question
        Question question(true);

        // Generate image message
        bwi_msgs::GetNextImageResponse nextImage = getQuestionImg();
        qReq.image = nextImage.img;
        qReq.point_cloud = nextImage.pc;
        qReq.pose = nextImage.pose;
        qReq.timeout = 100;
        qReq.question = question.question;

        // Send image message to `ask_location`
        askerClient.call(qReq, qRes);

        // Say `thank you!` after getting an answer, only if someone actually answered.
        if(qRes.answers.size() != 0)
            pub.publish(thanksSound);

        if(qRes.answers.size() > 0) {
            ROS_INFO_STREAM("Answer: \"" << qRes.answers[0] << '"');

            bwi_msgs::SaveImageResponseResponse saveRes;
            bwi_msgs::SaveImageResponseRequest saveReq;

            saveReq.base_name = nextImage.base_name;
            saveReq.location = locRes.locationName;
            saveReq.question_id = question.id;

            update_sarsa_agent(locRes.locationName, actionUpdateReq.QUESTION_ANSWERED);
        }
        else {
          ROS_INFO("No answer provided!");
          update_sarsa_agent(locRes.locationName, actionUpdateReq.QUESTION_TIMEOUT);
        }
    }
}

bwi_msgs::GetNextImageResponse getQuestionImg() {
    bwi_msgs::GetNextImageRequest req;
    bwi_msgs::GetNextImageResponse res;

    nextImageClient.call(req, res);

    return res;
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

void update_sarsa_agent(std::string location, int result) {
    bwi_msgs::ActionUpdateRequest actionUpdateReq;
    bwi_msgs::ActionUpdateResponse actionUpdateRes;

    actionUpdateReq.time.time_seconds = day_time_seconds();
    actionUpdateReq.time.day_of_week = day_week();
    actionUpdateReq.current_location = location;
    actionUpdateReq.result = result;

    updateSarsaClient.call(actionUpdateReq, actionUpdateRes);
}