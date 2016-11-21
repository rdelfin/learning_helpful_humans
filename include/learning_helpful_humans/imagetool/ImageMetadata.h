//
// Created by rdelfin on 11/17/16.
//

#ifndef PROJECT_IMAGEMETADATA_HPP
#define PROJECT_IMAGEMETADATA_HPP


#include <boost/uuid/uuid.hpp>

#include <geometry_msgs/Pose.h>

#include <json/json.hpp>

#include <string>


using json = nlohmann::json;

struct Answer {
    int questionId;
    std::string answer;
    int next;
    bool answered;
    unsigned long timestamp;
    std::string location;
};


/**
 * The JSON file format for this object looks as follows:
 * {
 *   "<UUID>": {
 *     "answers": {
 *       "count": "<int>",
 *       "list": [
 *         {
 *           "q": "<int>",
 *           "a": "<string>",
 *           "next": "<int>",
 *           "answered": "boolean",
 *           "time": "<ulong timestamp>",
 *           "location": "<string identifier>"
 *         }, ...
 *       ]
 *     },
 *     "pose": {
 *       "x": "<float>",
 *       "y": "<float>",
 *       "z": "<float>",
 *       "rx": "<float>",
 *       "ry": "<float>",
 *       "rz": "<float>",
 *       "rw": "<float>"
 *     }
 *   }
 * }
 */
class ImageMetadata {
public:
    ImageMetadata(const std::string& jsonData);
    ImageMetadata(const json& jsonData);
    ImageMetadata(const boost::uuids::uuid& indentifier, const geometry_msgs::Pose& pose, const std::vector<Answer>& answers);

    ~ImageMetadata();
private:
    boost::uuids::uuid identifier;
    geometry_msgs::Pose pose;
    std::vector<Answer> answers;
};


#endif //PROJECT_IMAGEMETADATA_HPP
