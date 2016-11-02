//
// Created by rdelfin on 11/1/16.
//

#ifndef PROJECT_POSTIMAGE_H
#define PROJECT_POSTIMAGE_H

#include <string>

class PostImageRequest {
public:
    PostImageRequest(uint8_t* jpegData, long len, std::string name);
    PostImageRequest(uint8_t* jpegData, long len);
    bool perform();

    ~PostImageRequest();
private:
    uint8_t* data;
    long len;
    std::string name;
    std::string server, imageroot;
};

#endif
