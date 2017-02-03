//
// Created by rdelfin on 11/22/16.
//

#include <learning_helpful_humans/request/GetFieldValue.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>
#include "learning_helpful_humans/imagetool/RandomImagePolicy.h"

#include <ros/ros.h>

RandomImagePolicy::RandomImagePolicy()
    : gen() {
    GetFieldValue getAllImages("imagedata.json");
    json data = getAllImages.performAsJson();

    for(auto it = data.begin(); it != data.end(); ++it)
        ids.push_back(boost::lexical_cast<boost::uuids::uuid>(it.key()));

    indexRand = std::uniform_int_distribution<size_t>(0, ids.size());
}

boost::uuids::uuid RandomImagePolicy::getNextImage() {
    // Picks a UUID at random and returns
    return ids[indexRand(gen)];
}

RandomImagePolicy::~RandomImagePolicy() {

}
