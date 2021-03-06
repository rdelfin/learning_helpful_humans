/*
 * Copyright 2017 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include <iterator>

#include <learning_helpful_humans/offline/ImageCache.h>

#include <ros/ros.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

ImageCache::ImageCache() {
    
}

ImageCache::ImageCache(ImagePickerPolicy* policy, size_t cachedImages)
    : policy(policy), cachedImages(cachedImages), write_all(false) {
    updateCache();
}

void ImageCache::updateCache() {
    //ROS_INFO_STREAM("UPDATING CACHE");
    
    cacheMutex.lock();
    while(readCache.size() < cachedImages) {
        DatabaseImage img = policy->getNextImage();
        ROS_INFO_STREAM("Adding image with id {" <<  boost::uuids::to_string(img.getIdentifier()) << "}");
        readCache.push_back(img);
        readCacheFetched.push_back(false);
    }
    cacheMutex.unlock();
    
    cacheMutex.lock();
    for(int i = 0; i < readCacheFetched.size(); i++) {
        if(!readCacheFetched[i]) {
            bool success = readCache[i].fetch();
            readCacheFetched[i] = success;
            
            if(success)
                ROS_INFO_STREAM("Successfully fetched image with ID {" << boost::uuids::to_string(readCache[i].getIdentifier()) <<"}");
        }
    }
    cacheMutex.unlock();
}

const DatabaseImage& ImageCache::getNextImage() {
    cacheMutex.lock();
    
    // Find a fetched image
    size_t i = 0;
    for(i = 0; i < readCacheFetched.size() && !readCacheFetched[i]; i++);
    ROS_INFO_STREAM("Getting image at index " << i);
    
    
    if(i < readCacheFetched.size()) {
        DatabaseImage img = readCache[i];
        readCache.erase(readCache.begin()+i);
        readCacheFetched.erase(readCacheFetched.begin()+i);
        imageCache.push_back(img);
        DatabaseImage& imgRef = imageCache.back();
        
        cacheMutex.unlock();
        
        ROS_INFO_STREAM("\tAssociated ID: {" << boost::uuids::to_string(imgRef.getIdentifier()) <<"}");
        
        return imgRef;
    } else {
        cacheMutex.unlock();
        throw "No fetched image found";
    }
    
    
}

bool ImageCache::addAnswer(boost::uuids::uuid imageId, Answer answer) {
    cacheMutex.lock();
    
    auto locIt = imageCache.begin();
    for(locIt = imageCache.begin();
        locIt->getIdentifier() != imageId && locIt != imageCache.end();
        ++locIt);
    
    // Image not found in the cache. Fetch from *THE INTERNET*
    if(locIt == imageCache.end()) {
        // Fetch Image
        DatabaseImage img(imageId);
        img.fetch();
        
        // Add to cache and get iterator pointer
        imageCache.push_back(img);
        locIt = std::prev(imageCache.end());
    }
    
    // Create copy for write cache and add answer
    DatabaseImage img = *locIt;
    img.addAnswer(answer);
    
    // Add to both caches
    *locIt = img;
    pendingWrites.push_back(img);
    
    cacheMutex.unlock();
    
    return true;
}

bool ImageCache::addNewImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud) {
    cacheMutex.lock();
    
    // Create image object
    DatabaseImage newImage(imageData, metadata, pointCloud, true);
    
    // Save to appropriate caches
    pendingWrites.push_back(newImage);
    imageCache.push_back(newImage);
    
    cacheMutex.unlock();
    
    return true;
}

void ImageCache::update() {
    updateCache();
    
    
    // Write out as neccessary
    cacheMutex.lock();
    bool status = true;
    while(status && !pendingWrites.empty()) {
        status = pendingWrites.front().post();
        if(status)
            pendingWrites.pop_front();
    }
    cacheMutex.unlock();
}

ImageCache::~ImageCache() {
    write_all = true;
    update();
}
