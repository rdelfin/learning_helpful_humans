//
// Created by rdelfin on 11/1/16.
//

#include <learning_helpful_humans/request/PostImageRequest.h>
#include <learning_helpful_humans/bytestream.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <sstream>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

PostImageRequest::PostImageRequest(uint8_t* jpegData, size_t len, std::string name, std::string mime_type, bool generate)
    : name(name), data(jpegData), mime_type(mime_type), len(len),
      server("https://firebasestorage.googleapis.com"),
      imageroot("v0/b/robotimages-dacc9.appspot.com/o"),
      postFields("alt=media&token=5bd2e983-3818-4721-b3f2-03e6677e9278") {

    if(generate) {
        // Generate UUID to represent file
        boost::uuids::random_generator generator;
        boost::uuids::uuid nameUuid = generator();

        // Store onto stream and set to name
        std::stringstream fileNameStream;
        fileNameStream << boost::uuids::to_string(nameUuid) << name;
        this->name = fileNameStream.str();
    }
}


bool PostImageRequest::perform() {
    std::stringstream urlss;
    urlss << server << "/" << imageroot << "/" << name << "?" << postFields;
    std::string url(urlss.str());

    try {
        curlpp::Cleanup myCleanup;
        curlpp::Easy req;

        char buf[50];

        // Setup Headers and add content header
        std::list<std::string> headers;
        headers.push_back("Content-Type: " + mime_type);
        sprintf(buf, "Content-Length: %lu", len);
        headers.push_back(buf);

        // Create data stream
        imemstream dataStream(data, len);

        // POST field

        req.setOpt(new curlpp::options::ReadStream(&dataStream));
        req.setOpt(new curlpp::options::InfileSize(len));
        req.setOpt(new curlpp::options::Post(true));
        req.setOpt(new curlpp::options::HttpHeader(headers));
        req.setOpt(new curlpp::options::Url(url));
        req.setOpt(new curlpp::options::NoSignal(true));
        req.setOpt(new curlpp::options::Timeout(2));

        req.perform();

        return true;

    } catch(curlpp::RuntimeError & e) {
        std::cerr << "Runtime error posting image request to id " << name << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    } catch(curlpp::LogicError & e) {
        std::cerr << "Logic error posting image request to id " << name << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    } catch(...) {
        std::cerr << "Unknown error posting image request to id " << name << std::endl;
        return false;
    }
}

std::string PostImageRequest::getName() {
    return name;
}

PostImageRequest::~PostImageRequest() {

}