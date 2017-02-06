/**
  * Created by rdelfin on 10/18/16.
  *
  * This drives the entire asking mechanism. Contains the overall logic: grab a location, go to it,
  * trigger the question asking, and save it onto the database. Uses all other nodes for all this
  * task, so it does very little work by itself.
  */

#include <ros/ros.h>

#include <bwi_msgs/NextLocation.h>
#include <bwi_msgs/ImageQuestion.h>
#include <sound_play/sound_play.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <learning_helpful_humans/Question.h>
#include <bwi_msgs/GetNextImage.h>
#include <bwi_msgs/SaveImageResponse.h>

bwi_msgs::GetNextImageResponse getQuestionImg();

ros::ServiceClient nextImageClient;
ros::ServiceClient saveResponseClient;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "location_asker_node");

    ros::NodeHandle nh;

    ros::ServiceClient askerClient = nh.serviceClient<bwi_msgs::ImageQuestion>("ask_location");
    ros::ServiceClient locationClient = nh.serviceClient<bwi_msgs::NextLocation>("next_question_location");
    nextImageClient = nh.serviceClient<bwi_msgs::GetNextImage>("image_tool/next");
    saveResponseClient = nh.serviceClient<bwi_msgs::SaveImageResponse>("image_tool/save_response");

    ros::Publisher pub = nh.advertise<sound_play::SoundRequest>("robotsound", 10);

    // Ensure the node does not start until all other node dependencies are up.
    askerClient.waitForExistence();
    locationClient.waitForExistence();
    nextImageClient.waitForExistence();
    saveResponseClient.waitForExistence();

    ros::Rate r(10);

    bwi_msgs::NextLocationRequest locReq;
    bwi_msgs::NextLocationResponse locRes;
    bwi_msgs::ImageQuestionRequest qReq;
    bwi_msgs::ImageQuestionResponse qRes;
    sound_play::SoundRequest helpSound;
    sound_play::SoundRequest thanksSound;

    helpSound.sound = thanksSound.sound = sound_play::SoundRequest::SAY;
    helpSound.command = thanksSound.command = sound_play::SoundRequest::PLAY_ONCE;
    helpSound.arg = "Hello! Can you help me by answering a quick question?";
    thanksSound.arg = "Thank you!";

    while(ros::ok()) {

        locationClient.call(locReq, locRes);

        if(locRes.success)
            ROS_WARN_STREAM("FAILED TO GO TO NEXT LOCATION.");
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
        qReq.timeout = 300;
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
        }
        else
          ROS_INFO("No answer provided!");
    }
}

bwi_msgs::GetNextImageResponse getQuestionImg() {
    bwi_msgs::GetNextImageRequest req;
    bwi_msgs::GetNextImageResponse res;

    nextImageClient.call(req, res);

    return res;
}
