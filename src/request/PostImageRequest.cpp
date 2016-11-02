//
// Created by rdelfin on 11/1/16.
//

#include <learning_helpful_humans/requests/PostImageRequest.h>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <sstream>
#include <learning_helpful_humans/bytestream.h>

PostImageRequest::PostImageRequest(uint8_t* jpegData, long len, std::string name)
    : name(name), data(jpegData), len(len),
      server("https://firebasestorage.googleapis.com"),
      imageroot("v0/b/robotimages-dacc9.appspot.com/o") {

}

PostImageRequest::PostImageRequest(uint8_t* jpegData, long len)
    : name(name), data(jpegData), len(len),
      server("https://firebasestorage.googleapis.com"),
      imageroot("v0/b/robotimages-dacc9.appspot.com/o") {

}


bool PostImageRequest::perform() {
    std::stringstream urlss;
    urlss << server << "/" << imageroot << "/" << name;
    std::string url(urlss.str());

    try {
        curlpp::Cleanup myCleanup;
        curlpp::Easy req;

        char buf[50];

        // Setup Headers and add content header
        std::list<std::string> headers;
        headers.push_back("Content-Type: image/jpeg");
        sprintf(buf, "Content-Length: %ld", len);
        headers.push_back(buf);

        // Create data stream
        memstream dataStream(data, len);

        req.setOpt(new curlpp::options::ReadStream(&dataStream));
        req.setOpt(new curlpp::options::InfileSize(len));
        req.setOpt(new curlpp::options::Upload(true));
        req.setOpt(new curlpp::options::HttpHeader(headers));
        req.setOpt(new curlpp::options::Url(url));

        req.perform();

        return true;

    } catch(curlpp::RuntimeError & e) {
        std::cerr << e.what() << std::endl;
        return false;
    } catch(curlpp::LogicError & e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

PostImageRequest::~PostImageRequest() {

}