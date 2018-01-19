//
// Created by rdelfin on 11/11/16.
//

#ifndef PROJECT_APPENDFIELDVALUE_HPP
#define PROJECT_APPENDFIELDVALUE_HPP

#include <string>
#include <json/json.hpp>

using json = nlohmann::json;


class AppendFieldValue {
public:
    AppendFieldValue(std::string path, json value);
    AppendFieldValue(std::string path, std::string value);
    AppendFieldValue(std::string path, int value);

    bool perform();

    ~AppendFieldValue();

private:
    std::string value, path;
    std::string server;
};


#endif //PROJECT_APPENDFIELDVALUE_HPP
