//
// Created by rdelfin on 11/8/16.
//

#include "learning_helpful_humans/request/PostFieldValue.h"

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <string>
#include <json/json.hpp>
#include <learning_helpful_humans/bytestream.h>

using json = nlohmann::json;

PostFieldValue::PostFieldValue(std::string path, json value)
    : path(path), server("robotimages-dacc9.firebaseio.com"), value(value.dump()) {

}

PostFieldValue::PostFieldValue(std::string path, std::string value)
    : path(path), server("robotimages-dacc9.firebaseio.com"), value("\"" + value + "\"") {

}

PostFieldValue::PostFieldValue(std::string path, int value)
    : path(path), server("robotimages-dacc9.firebaseio.com"), value(std::to_string(value)) {

}

bool PostFieldValue::perform() {
    std::stringstream urlStream;
    urlStream << server << "/" << path;
    std::string url = urlStream.str();

    try {
        char buf[255];

        curlpp::Cleanup myCleanup;
        curlpp::Easy req;

        imemstream dataStream((uint8_t*) value.c_str(), value.length());


        // Setup Headers and add content header
        std::list<std::string> headers;
        headers.push_back("Content-Type: application/json");
        sprintf(buf, "Content-Length: %lu", value.length());
        headers.push_back(buf);

        // POST field
        req.setOpt(new curlpp::options::Url(url));
        req.setOpt(new curlpp::options::Put(true));

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

PostFieldValue::~PostFieldValue() {

}
