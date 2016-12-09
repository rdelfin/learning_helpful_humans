//
// Created by rdelfin on 11/15/16.
//

#include "learning_helpful_humans/request/GetImage.h"

#include <boost/uuid/uuid_io.hpp>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

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

        req.perform();

        std::string resultString = resultStream.str();

        return std::vector<uint8_t>(resultString.begin(), resultString.end());

    } catch(curlpp::RuntimeError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    } catch(curlpp::LogicError & e) {
        std::cerr << e.what() << std::endl;
        return json();
    }
}

cv::Mat GetImage::performImage() {
    std::vector<uint8_t> data = performRaw();
    cv::InputArray dataArray(data);

    return cv::imdecode(data, cv::IMREAD_COLOR);
}

GetImage::~GetImage() {

}
