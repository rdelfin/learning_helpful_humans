//
// Created by rdelfin on 11/8/16.
//

#ifndef PROJECT_GETFIELDVALUE_HPP
#define PROJECT_GETFIELDVALUE_HPP

#include <string>
#include <json/json.hpp>

using json = nlohmann::json;

class GetFieldValue {
public:
    GetFieldValue(std::string path);

    json perform();

    ~GetFieldValue();
private:
    std::string path;
     std::string server;
};


#endif //PROJECT_GETFIELDVALUE_HPP
