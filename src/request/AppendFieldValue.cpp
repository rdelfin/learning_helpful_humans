//
// Created by rdelfin on 11/11/16.
//

#include "learning_helpful_humans/request/AppendFieldValue.h"
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <string>
#include <json/json.hpp>
#include <learning_helpful_humans/bytestream.h>

using json = nlohmann::json;

AppendFieldValue::AppendFieldValue(std::string path, json value)
        : path(path), server("https://robotimages-dacc9.firebaseio.com"), value(value.dump()) {

}

AppendFieldValue::AppendFieldValue(std::string path, std::string value)
        : path(path), server("https://robotimages-dacc9.firebaseio.com"), value("\"" + value + "\"") {

}

AppendFieldValue::AppendFieldValue(std::string path, int value)
        : path(path), server("https://robotimages-dacc9.firebaseio.com"), value(std::to_string(value)) {

}

bool AppendFieldValue::perform() {
    std::stringstream urlStream;
    urlStream << server << "/" << path;
    std::string url = urlStream.str();

    try {
        char buf[255];

        curlpp::Cleanup myCleanup;
        curlpp::Easy req;


        std::stringstream result;
        imemstream dataStream((uint8_t*) value.c_str(), value.length());


        // Setup Headers and add content header
        std::list<std::string> headers;
        headers.push_back("Content-Type: application/json");
        sprintf(buf, "Content-Length: %lu", value.length());
        headers.push_back(buf);
        headers.push_back("X-HTTP-Method-Override: PATCH");

        // POST field
        req.setOpt(new curlpp::options::Url(url));
        req.setOpt(new curlpp::options::Post(true));
        req.setOpt(new curlpp::options::HttpHeader(headers));
        req.setOpt(new curlpp::options::ReadStream(&dataStream));
        req.setOpt(new curlpp::options::InfileSize(value.length()));
        req.setOpt(new curlpp::options::WriteStream(&result));
        req.setOpt(new curlpp::options::NoSignal(true));
        req.setOpt(new curlpp::options::Timeout(2));

        req.perform();

        std::string resultString = result.str();
        std::cout << "Result: " << resultString << std::endl;
        json resultJson = json::parse(resultString);
        return true;

    } catch(curlpp::RuntimeError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    } catch(curlpp::LogicError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    }
}

AppendFieldValue::~AppendFieldValue() {

}
