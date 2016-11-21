//
// Created by rdelfin on 11/17/16.
//

#include <boost/lexical_cast.hpp>
#include "learning_helpful_humans/imagetool/ImageMetadata.h"

ImageMetadata::ImageMetadata(const std::string& jsonData)
    : ImageMetadata(json::parse(jsonData))
{
}

ImageMetadata::ImageMetadata(const json& jsonData) {
    // Obtain the UUID of the image metadata
    auto parent = jsonData.begin();
    if(parent != jsonData.end()) {
        std::string idString = parent.key();
        identifier = boost::lexical_cast<boost::uuids::uuid>(idString);

        json imageBody = parent.value();

        // Load in the pose (fastest part)
        json poseJson = imageBody["pose"];
        pose.position.x = poseJson["x"];
        pose.position.x = poseJson["y"];
        pose.position.x = poseJson["z"];
        pose.orientation.x = poseJson["rx"];
        pose.orientation.y = poseJson["ry"];
        pose.orientation.z = poseJson["rz"];
        pose.orientation.w = poseJson["rw"];

        // Load in answers
        json answersJson = json(imageBody["answers"])["list"];

        for(auto ansIt = answersJson.begin(); ansIt != answersJson.end(); ++ansIt) {
            json answerJson = ansIt.value();

            Answer answer;
            answer.questionId = answerJson["q"];
            if(answerJson.count("a") > 0)
                answer.answer = answerJson["a"];

            if(answerJson.count("next") > 0)
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

ImageMetadata::ImageMetadata(const boost::uuids::uuid& indentifier, const geometry_msgs::Pose& pose, const std::vector<Answer>& answers) :
    identifier(identifier), pose(pose), answers(answers) {
}


ImageMetadata::~ImageMetadata() {

}


