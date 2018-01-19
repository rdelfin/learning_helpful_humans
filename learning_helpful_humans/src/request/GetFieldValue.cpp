//
// Created by rdelfin on 11/8/16.
//

#include <learning_helpful_humans/request/GetFieldValue.h>
#include <learning_helpful_humans/request/TimeoutException.h>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <sstream>

#include <json/json.hpp>

#include <ros/ros.h>

using json = nlohmann::json;


GetFieldValue::GetFieldValue(std::string path)
        : path(path), server("https://robotimages-dacc9.firebaseio.com") {

}

json GetFieldValue::performAsJson() {
    std::stringstream urlStream;
    urlStream << server << "/" << path;
    std::string url = urlStream.str();


    try {
        curlpp::Cleanup myCleanup;
        curlpp::Easy req;

        std::stringstream resultStream;

        // POST field
        req.setOpt(new curlpp::options::WriteStream(&resultStream));
        req.setOpt(new curlpp::options::Url(url));
        req.setOpt(new curlpp::options::NoSignal(true));

        req.perform();

        std::string result = resultStream.str();

        return json::parse(result);

    } catch(curlpp::RuntimeError & e) {
        ROS_ERROR_STREAM("Runtime error when getting field at path \"" << path << "\"");
        ROS_ERROR_STREAM(e.what());
        return json();
    } catch(curlpp::LogicError & e) {
        ROS_ERROR_STREAM("Logic error when getting field at path \"" << path << "\"");
        ROS_ERROR_STREAM(e.what());
        return json();
    } catch(...) {
        ROS_ERROR_STREAM("Unknown error getting field at path \"" << path << "\"");
        return json();
    }
}

std::string GetFieldValue::performAsString() {
    std::stringstream urlStream;
    urlStream << server << "/" << path;
    std::string url = urlStream.str();

    try {
        curlpp::Cleanup myCleanup;
        curlpp::Easy req;

        // POST field
        req.setOpt(new curlpp::options::Url(url));
        req.setOpt(new curlpp::options::NoSignal(true));
        req.setOpt(new curlpp::options::Timeout(10));

        req.perform();

        std::stringstream result;
        result << req;

        return result.str();

    } catch(curlpp::RuntimeError & e) {
        // Assume this is timeout
        throw TimeoutException(10);
    } catch(curlpp::LogicError & e) {
        ROS_ERROR_STREAM("Logic error when getting field at path \"" << path << "\"");
        ROS_ERROR_STREAM(e.what());
        return "";
    } catch(...) {
        ROS_ERROR_STREAM("Unknown error getting field at path \"" << path << "\"");
        return "";
    }
}

GetFieldValue::~GetFieldValue() {

}
