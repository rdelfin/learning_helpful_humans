//
// Created by rdelfin on 11/17/16.
//

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <learning_helpful_humans/request/AppendFieldValue.h>
#include "learning_helpful_humans/imagetool/ImageMetadata.h"

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
    json answerList, answersObject, poseJson, jsonBody, jsonResult;
    int count = 0;

    // Load up the list of answers
    for(auto it = answers.begin(); it != answers.end(); ++it) {
        json answerJson;
        answerJson["q"] = it->questionId;
        if(it->answered) // Only set the answer object if answered is true
            answerJson["a"] = it->answer;
        if(it->next >= 0) // Only set the next object if next is non-negative
            answerJson["next"] = it->next;
        answerJson["answered"] = it->answered;
        answerJson["time"] = it->timestamp;
        answerJson["location"] = it->location;

        answerList.push_back(answerJson);
        count++;
    }
    answersObject["count"] = count;
    answersObject["list"] = answerList;

    // Loads in the pose
    poseJson["x"] = pose.position.x;
    poseJson["y"] = pose.position.y;
    poseJson["z"] = pose.position.z;
    poseJson["rx"] = pose.orientation.x;
    poseJson["ry"] = pose.orientation.y;
    poseJson["rz"] = pose.orientation.z;
    poseJson["rw"] = pose.orientation.w;

    // Constructs the final JSON object
    jsonBody["pose"] = poseJson;
    jsonBody["answers"] = answersObject;
    jsonResult[boost::uuids::to_string(identifier)] = jsonBody;

    // Send the data to the server
    AppendFieldValue appendImageData("imagedata.json", jsonResult);
    return appendImageData.perform();
}


ImageMetadata::~ImageMetadata() {

}


