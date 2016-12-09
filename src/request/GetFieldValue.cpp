//
// Created by rdelfin on 11/8/16.
//

#include <learning_helpful_humans/request/GetFieldValue.h>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <sstream>

#include <json/json.hpp>

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

        req.perform();

        std::string result = resultStream.str();

        return json::parse(result);

    } catch(curlpp::RuntimeError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    } catch(curlpp::LogicError & e) {
        std::cerr << e.what() << std::endl;
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

        req.perform();

        std::stringstream result;
        result << req;

        return result.str();

    } catch(curlpp::RuntimeError & e) {
        std::cerr << e.what() << std::endl;
        return "";
    } catch(curlpp::LogicError & e) {
        std::cerr << e.what() << std::endl;
        return "";
    }
}

GetFieldValue::~GetFieldValue() {

}