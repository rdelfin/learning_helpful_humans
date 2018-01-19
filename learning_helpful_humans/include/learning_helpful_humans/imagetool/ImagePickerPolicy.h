//
// Created by rdelfin on 11/22/16.
//

#ifndef PROJECT_IMAGEPICKERPOLICY_HPP
#define PROJECT_IMAGEPICKERPOLICY_HPP

#include <boost/uuid/uuid.hpp>

class ImagePickerPolicy {
public:
    ImagePickerPolicy() { }
    virtual boost::uuids::uuid getNextImage() = 0;
    virtual ~ImagePickerPolicy() { }
private:
};


#endif //PROJECT_IMAGEPICKERPOLICY_HPP
