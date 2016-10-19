//
// Created by rdelfin on 10/18/16.
//

#include <ros/ros.h>

#include <bwi_msgs/NextLocation.h>
#include <bwi_msgs/ImageQuestion.h>
#include <sound_play/sound_play.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

sensor_msgs::Image getQuestionImg();

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "location_asker_node");

    ros::NodeHandle nh;

    ros::ServiceClient askerClient = nh.serviceClient<bwi_msgs::ImageQuestion>("ask_location");
    ros::ServiceClient locationClient = nh.serviceClient<bwi_msgs::NextLocation>("next_question_location");

    ros::Publisher pub = nh.advertise<sound_play::SoundRequest>("robotsound", 10);

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

        qReq.image = getQuestionImg();
        askerClient.call(qReq, qRes);

        ROS_INFO_STREAM("Answer: \"" << qRes.answers[0] << '"');
    }
}

sensor_msgs::Image getQuestionImg() {
    // We'll be returning a fixed image bc why not
    cv::Mat image = cv::imread("/home/users/rdelfin/Downloads/img_1474998260668572241.png", CV_LOAD_IMAGE_COLOR);

    std_msgs::Header header;
    header.stamp = header.stamp.now();
    cv_bridge::CvImage bridgeImg(header, sensor_msgs::image_encodings::RGB8, image);
    return *bridgeImg.toImageMsg();
}
