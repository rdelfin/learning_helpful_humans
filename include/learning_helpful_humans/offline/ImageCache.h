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

#ifndef IMAGECACHE_H
#define IMAGECACHE_H

#include <boost/uuid/uuid.hpp>

#include <queue>
#include <deque>
#include <mutex>

#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <learning_helpful_humans/imagetool/ImageMetadata.h>
#include <learning_helpful_humans/imagetool/ImagePickerPolicy.h>

class ImageCache
{
public:
    ImageCache();
    ImageCache(ImagePickerPolicy* policy, size_t cachedImages);
    
    const DatabaseImage& getNextImage();
    bool addAnswer(boost::uuids::uuid imageId, Answer answer);
    bool addNewImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud);
    
    void update();
    
    ~ImageCache();
    
private:
    std::deque<DatabaseImage> readCache;
    std::deque<DatabaseImage> pendingWrites;
    
    std::deque<DatabaseImage> imageCache;
    
    ImagePickerPolicy* policy;
    size_t cachedImages;
    
    void updateCache();
    
    std::mutex cacheMutex;
    
    bool write_all;
};

#endif // IMAGECACHE_H
