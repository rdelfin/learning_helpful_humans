//
// Created by rdelfin on 11/1/16.
//

#ifndef PROJECT_POSTIMAGE_H
#define PROJECT_POSTIMAGE_H

#include <string>

class PostImageRequest {
public:
    /**
     * Default constructor for PostImageRequest
     * @param jpegData Starting address of the JPEG data to send
     * @param len Length of the data array to read
     * @param name Name of the file or, if generate is true, the extension of the file
     * @param generate When set to true, name will be used as an extension and a randomly assigned name will be given
     */
    PostImageRequest(uint8_t* jpegData, size_t len, std::string name, bool generate = false);
    bool perform();

    std::string getName();

    ~PostImageRequest();
private:
    uint8_t* data;
    size_t len;
    std::string name;
    std::string server, imageroot, postFields;
};

#endif
