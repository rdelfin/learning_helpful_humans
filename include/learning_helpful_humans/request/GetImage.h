//
// Created by rdelfin on 11/15/16.
//

#ifndef PROJECT_GETIMAGE_HPP
#define PROJECT_GETIMAGE_HPP

#include <string>
#include <json/json.hpp>
#include <boost/uuid/uuid.hpp>

#include <opencv2/opencv.hpp>

#include <ctype.h>

using json = nlohmann::json;

class GetImage {
public:
    GetImage();
    GetImage(boost::uuids::uuid identifier, std::string extension, std::string server);

    std::vector<uint8_t> performRaw();
    cv::Mat performImage();

    ~GetImage();
private:
    boost::uuids::uuid identifier;
    std::string server, extension;
};

#endif //PROJECT_GETIMAGE_HPP
