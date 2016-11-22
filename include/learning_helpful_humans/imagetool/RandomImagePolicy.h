//
// Created by rdelfin on 11/22/16.
//

#ifndef PROJECT_RANDOMIMAGEPOLICY_HPP
#define PROJECT_RANDOMIMAGEPOLICY_HPP

#include <learning_helpful_humans/imagetool/ImagePickerPolicy.h>
#include <random>


class RandomImagePolicy : public ImagePickerPolicy {
public:
    RandomImagePolicy();
    virtual boost::uuids::uuid getNextImage();
    ~RandomImagePolicy();
private:
    std::vector<boost::uuids::uuid> ids;
    std::default_random_engine gen;
    std::uniform_int_distribution<size_t> rand;
};


#endif //PROJECT_RANDOMIMAGEPOLICY_HPP
