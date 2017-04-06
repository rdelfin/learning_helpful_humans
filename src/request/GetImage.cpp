//
// Created by rdelfin on 11/15/16.
//

#include "learning_helpful_humans/request/GetImage.h"

#include <boost/uuid/uuid_io.hpp>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <ros/ros.h>

GetImage::GetImage()
        : server("https://firebasestorage.googleapis.com"),
         imageRoot("v0/b/robotimages-dacc9.appspot.com/o"),
          postFields("alt=media") {

}

GetImage::GetImage(boost::uuids::uuid identifier, std::string extension)
    : identifier(identifier), extension(extension),
      server("https://firebasestorage.googleapis.com"),
      imageRoot("v0/b/robotimages-dacc9.appspot.com/o"),
      postFields("alt=media") {

}

std::vector<uint8_t> GetImage::performRaw() {
    std::stringstream urlStream;
    urlStream << server << "/" << imageRoot << "/"
              << boost::uuids::to_string(identifier) << "." << extension
              << "?" << postFields;
    std::string url = urlStream.str();

    try {
        std::stringstream resultStream;
        curlpp::Cleanup myCleanup;
        curlpp::Easy req;

        req.setOpt(new curlpp::options::WriteStream(&resultStream));
        req.setOpt(new curlpp::options::Url(url));
        req.setOpt(new curlpp::options::NoSignal(true));
        req.setOpt(new curlpp::options::Timeout(2));

        req.perform();

        std::string resultString = resultStream.str();

        return std::vector<uint8_t>(resultString.begin(), resultString.end());

    } catch(curlpp::RuntimeError & e) {
        // Assume this is timeout.
        ROS_ERROR_STREAM("Runtime error when getting an image with name \"" << boost::uuids::to_string(identifier) << "\"");
        ROS_ERROR_STREAM(e.what());
        return std::vector<uint8_t>();
    } catch(curlpp::LogicError & e) {
        ROS_ERROR_STREAM("Logic error when getting an image with name \"" << boost::uuids::to_string(identifier) << "\"");
        ROS_ERROR_STREAM(e.what());
        return std::vector<uint8_t>();
    } catch(...) {
        ROS_ERROR_STREAM("Unknown error getting image with name \"" << boost::uuids::to_string(identifier) << "\"");
        return std::vector<uint8_t>();
    }
}

cv::Mat GetImage::performImage() {
    std::vector<uint8_t> data = performRaw();
    cv::InputArray dataArray(data);

    return cv::imdecode(data, cv::IMREAD_COLOR);
}

GetImage::~GetImage() {

}
