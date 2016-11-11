//
// Created by rdelfin on 11/8/16.
//

#ifndef PROJECT_POSTFIELDVALUE_HPP
#define PROJECT_POSTFIELDVALUE_HPP

#include <string>
#include <json/json.hpp>

using json = nlohmann::json;

class PostFieldValue {
public:
    PostFieldValue(std::string path, json value);
    PostFieldValue(std::string path, std::string value);
    PostFieldValue(std::string path, int value);

    bool perform();

    ~PostFieldValue();

private:
    std::string value, path;
    std::string server;
};


#endif //PROJECT_POSTFIELDVALUE_HPP
