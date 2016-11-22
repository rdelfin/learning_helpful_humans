//
// Created by rdelfin on 11/22/16.
//

#ifndef PROJECT_IMAGEPICKERPOLICY_HPP
#define PROJECT_IMAGEPICKERPOLICY_HPP


class ImagePickerPolicy {
public:
    ImagePickerPolicy();
    virtual UUID getNextImage();
    virtual ~ImagePickerPolicy();
private:
};


#endif //PROJECT_IMAGEPICKERPOLICY_HPP
