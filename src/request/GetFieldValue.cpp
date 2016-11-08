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
        : path(path), server("robotimages-dacc9.firebaseio.com") {

}

json GetFieldValue::perform() {
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

        return json::parse(result.str());

    } catch(curlpp::RuntimeError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    } catch(curlpp::LogicError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    }
}

GetFieldValue::~GetFieldValue() {

}