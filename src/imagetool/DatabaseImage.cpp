//
// Created by rdelfin on 11/11/16.
//

#include <learning_helpful_humans/imagetool/DatabaseImage.h>
#include <learning_helpful_humans/request/PostImageRequest.h>
#include <learning_helpful_humans/request/AppendFieldValue.h>
#include <learning_helpful_humans/request/GetFieldValue.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <boost/uuid/uuid_io.hpp>
#include <learning_helpful_humans/request/GetImage.h>
#include <tf/transform_datatypes.h>

#include <cstdlib>

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <boost/uuid/random_generator.hpp>

static boost::uuids::random_generator gen;

DatabaseImage::DatabaseImage() {

}

DatabaseImage::DatabaseImage(boost::uuids::uuid identifier)
    : identifier(identifier) {

}

DatabaseImage::DatabaseImage(const sensor_msgs::Image& imageData, ImageMetadata metadata, const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, bool newImage)
    : imageData(imageData), metadata(metadata), pointCloud(pointCloud) {
    // Special case: if newImage is true, then generate an ID for the image
    if(newImage) {
        this->metadata.identifier = gen();
        this->identifier = this->metadata.identifier;
    }
}

const boost::uuids::uuid& DatabaseImage::getIdentifier() {
    return identifier;
}


const sensor_msgs::Image& DatabaseImage::getImageData() {
    return imageData;
}

const ImageMetadata& DatabaseImage::getMetadata() {
    return metadata;
}

const pcl::PointCloud<pcl::PointXYZRGB>& DatabaseImage::getPointCloud() {
    return pointCloud;
}

void DatabaseImage::addAnswer(Answer a) {
    metadata.answers.push_back(a);
}


bool DatabaseImage::fetch() {
    // Fetch image
    GetImage getImg(identifier, "jpg");

    cv::Mat img = getImg.performImage();
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    imageData = *cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

    // Fetch metadata
    std::stringstream pathString;
    pathString << "imagedata/" << boost::uuids::to_string(identifier) << ".json";
    GetFieldValue poseGet(pathString.str());
    json metadataJson = poseGet.performAsJson();
    metadata = ImageMetadata(metadataJson, identifier);


    // Fetch PCL Image
    // Again, I find myself in contempt of all that is good in programming (see the post() method)
    GetImage getPcl(identifier, "pcd");
    std::vector<uint8_t> rawData = getPcl.performRaw();
    std::string stringPclData(rawData.begin(), rawData.end());

    // Write file out to temp, so it can be read in and my every instinct as a programmer can be destroyed
    std::string fileName = boost::uuids::to_string(metadata.identifier) + ".pcd";
    std::ofstream outPclFile(fileName);
    outPclFile << stringPclData;
    outPclFile.close();

    // Read file back into PCL object
    pcl::io::loadPCDFile(fileName, pointCloud);

    //--------------------------------
    //</end_atrocity_of_software_engineering>
    //--------------------------------

    return true;
}


bool DatabaseImage::post() {
    bool success;

    std::string basename = boost::uuids::to_string(this->metadata.identifier);

    cv_bridge::CvImagePtr imageBridge = cv_bridge::toCvCopy(imageData);
    std::vector<uchar> dataBuffer;
    cv::imencode(".jpg", imageBridge->image, dataBuffer); // Stores jpg data in dataBuffer

    PostImageRequest imagePost(&dataBuffer[0], dataBuffer.size(), basename + ".jpg", "image/jpeg", false); // Create the HTTP request
    success = imagePost.perform();                               // Post request to firebase

    // Check for failure and abort
    if(!success)
        return false;


    /* Write the point cloud. This is done by first writing out to a temporary file, reading in the contents of
     * said file, and writing them to the online database
     * I hold myself in personal contempt for this piece of code I wrote. Please find me and throw me into
     * a volcano. Thank you for your cooperation */
    std::string fileName = boost::uuids::to_string(metadata.identifier) + ".pcd";

    // Write to the file
    pcl::io::savePCDFileASCII (fileName, pointCloud);

    // Read said file back in (because PCL)
    std::ifstream pclReader(fileName);
    std::stringstream pclStream;
    pclStream << pclReader.rdbuf();
    pclReader.close();
    std::string pclData(pclStream.str());

    // Delete aforementioned file so it can return to the fires of hell from whence it came
    unlink(fileName.c_str());

    // Write out to Firebase in a similar fashion to the image
    PostImageRequest pointCloudPost((uint8_t*)pclData.c_str(), pclData.size(), basename + ".pcd", "text/plain");
    success = pointCloudPost.perform();


    // Check for failure and abort
    if(!success)
        return false;


    success = metadata.postUpdate();

    return success;
}

DatabaseImage::~DatabaseImage() {

}
