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

ImageCache::ImageCache()
{
    
}

ImageCache::ImageCache(ImagePickerPolicy* policy, size_t cachedImages)
    : policy(policy), cachedImages(cachedImages), write_all(false) {
    updateCache();
}

void ImageCache::updateCache() {
    
    while(readCache.size() < cachedImages) {
        DatabaseImage img = policy->getNextImage();
        
        cacheMutex.lock();
        readCache.push_back(img);
        cacheMutex.unlock();
    }
}

const DatabaseImage& ImageCache::getNextImage() {
    cacheMutex.lock();
    
    DatabaseImage img = readCache.front();
    readCache.pop_front();
    imageCache.push_back(img);
    DatabaseImage& imgRef = imageCache.back();
    
    cacheMutex.unlock();
    
    return imgRef;
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
    cacheMutex.lock();
    
    bool status = true;
    while(status && !pendingWrites.empty())
        status = pendingWrites.front().post();
    
    cacheMutex.unlock();
}

ImageCache::~ImageCache() {
    write_all = true;
    update();
}
