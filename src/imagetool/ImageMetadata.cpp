//
// Created by rdelfin on 11/17/16.
//

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <learning_helpful_humans/request/AppendFieldValue.h>
#include "learning_helpful_humans/imagetool/ImageMetadata.h"


char imageFormatString[] = R"(
{
    "%s": {
        "questions": {
            "count": 0,
            "list": []
        },
        "pose": {
            "x": %f,
            "y": %f,
            "z": %f,
            "rx": %f,
            "ry": %f,
            "rz": %f,
            "rw": %f
        }
    }
}
)";

ImageMetadata::ImageMetadata(const std::string& jsonData, boost::uuids::uuid id)
    : ImageMetadata(json::parse(jsonData), id)
{
}

ImageMetadata::ImageMetadata(const json& jsonData, boost::uuids::uuid id) {
    identifier = id;

    // Load in the pose (fastest part)
    json poseJson = jsonData["pose"];
    pose.position.x = poseJson["x"];
    pose.position.x = poseJson["y"];
    pose.position.x = poseJson["z"];
    pose.orientation.x = poseJson["rx"];
    pose.orientation.y = poseJson["ry"];
    pose.orientation.z = poseJson["rz"];
    pose.orientation.w = poseJson["rw"];

    // Load in answers
    if(json(jsonData["answers"]).count("list") > 0) {
        json answersJson = json(jsonData["answers"])["list"];

        for (auto ansIt = answersJson.begin(); ansIt != answersJson.end(); ++ansIt) {
            json answerJson = ansIt.value();

            Answer answer;
            answer.questionId = answerJson["q"];
            if (answerJson.count("a") > 0)
                answer.answer = answerJson["a"];

            if (answerJson.count("next") > 0)
                answer.next = answerJson["next"];
            else
                answer.next = -1;

            answer.answered = answerJson["answered"];
            answer.timestamp = answerJson["time"];
            answer.location = answerJson["location"];

            // Push the answer into the vector of answers
            answers.push_back(answer);
        }
    }
}

ImageMetadata::ImageMetadata(const boost::uuids::uuid& identifier, const geometry_msgs::Pose& pose, const std::vector<Answer>& answers) :
    identifier(identifier), pose(pose), answers(answers) {
}

bool ImageMetadata::postUpdate() {
    char buf[1024];
    snprintf(buf, 1024, imageFormatString, boost::uuids::to_string(identifier),
             pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
             pose.orientation.y, pose.orientation.z, pose.orientation.w);

    json imageData = json::parse(std::string(buf));

    AppendFieldValue appendImageData("imagedata", imageData);
    return appendImageData.perform();
}


ImageMetadata::~ImageMetadata() {

}


