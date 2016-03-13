#include "flagdetector.h"
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

// Offsets for red and blue flag detections
const int redDiff = 180;
const int blueDiff = 140;
const int blockSize = 5;
const int blueOffset = 30;
const int redOffset = 140;

double toRadians(double degrees) {
    return degrees / 180.0 * M_PI;
}

void FlagDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat orig = cv_ptr->image.clone();
    src = cv_ptr->image.clone();

    // Crops the image (removes sky)
    int topCrop = src.rows / 2 - 100;
    Rect roiNoSky(0, topCrop, src.cols, src.rows - topCrop);
    src = src(roiNoSky);

    vector<Point> redFlagPoints;
    vector<Point> blueFlagPoints;
    cerr << "hi" << endl;
    // Average red and blue values
    int redAvg = 0;
    int blueAvg = 0;
    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            Vec3b currentPixel = src.at<Vec3b>(i, j);
            redAvg += currentPixel[2];
            blueAvg += currentPixel[0];
        }
    }
    redAvg /= src.rows * src.cols;
    blueAvg /= src.rows * src.cols;
    cerr << to_string(redAvg) << endl;
    cerr << to_string(blueAvg) << endl;
    // Determine which points corresponds to red / blue flags.

    for (int i = 0; i < src.cols; i += blockSize) {
        for (int j = 0; j < src.rows; j += blockSize) {
            int redSquareAvg = 0;
            int blueSquareAvg = 0;
            int greenSquareAvg = 0;
            for (int k = i; k < i + blockSize && k < src.cols; k++) {
                for (int l = j; l < j + blockSize && j < src.rows; l++) {
                    Vec3b currentPixel = src.at<Vec3b>(l, k);
                    blueSquareAvg += currentPixel[0];
                    greenSquareAvg += currentPixel[1];
                    redSquareAvg += currentPixel[2];
                }
            }
            redSquareAvg /= blockSize * blockSize;
            blueSquareAvg /= blockSize * blockSize;
            greenSquareAvg /= blockSize * blockSize;

            if (blueSquareAvg - blueDiff > greenSquareAvg && blueSquareAvg - blueDiff > redSquareAvg && blueSquareAvg - blueAvg > blueOffset) {
                for (int k = i; k < i + blockSize && k < src.cols; k++) {
                    for (int l = j; l < j + blockSize && j < src.rows; l++) {
                        blueFlagPoints.push_back(Point(k, l));
                        cerr << to_string(redSquareAvg) + " " + to_string(greenSquareAvg) + " " + to_string(blueSquareAvg) << endl;
                        Vec3b currentPixel = src.at<Vec3b>(l, k);
                        currentPixel[0] = 0;
                        currentPixel[1] = 0;
                        currentPixel[2] = 0;
                        src.at<Vec3b>(l, k) = currentPixel;
                    }
                }
            } else if (redSquareAvg - redDiff > blueSquareAvg && redSquareAvg - redDiff > greenSquareAvg && redSquareAvg - redAvg > redOffset) {
                for (int k = i; k < i + blockSize && k < src.cols; k++) {
                    for (int l = j; l < j + blockSize && j < src.rows; l++) {
                        redFlagPoints.push_back(Point(k, l));
                        cerr << to_string(redSquareAvg) + " " + to_string(greenSquareAvg) + " " + to_string(blueSquareAvg) << endl;
                        Vec3b currentPixel = src.at<Vec3b>(l, k);
                        currentPixel[0] = 255;
                        currentPixel[1] = 255;
                        currentPixel[2] = 255;
                        src.at<Vec3b>(l, k) = currentPixel;
                    }
                }
            }
        }
    }

    // Convert the contours to a pointcloud
    // cloud = toPointCloud(allContours, orig.rows, orig.cols);

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image    = src;

    cv_ptr->image = src;
    _flag_filt_img.publish(cv_ptr->toImageMsg());
    _flag_thres.publish(out_msg.toImageMsg());
    // _flag_cloud.publish(cloud);
}

FlagDetector::FlagDetector(ros::NodeHandle &handle)
    : gaussian_size(7),
      _it(handle),
      tf_listener(handle) {
    _src_img = _it.subscribe("/stereo/right/image_raw", 1, &FlagDetector::img_callback, this);
    _flag_filt_img = _it.advertise("/flag_filt_img", 1);
    _flag_thres = _it.advertise("/flag_thres", 1);
    _flag_cloud = handle.advertise<PCLCloud>("/flag_cloud", 100);
}

PointCloud<PointXYZ>::Ptr FlagDetector::toPointCloud(vector<vector<Point>> &contours, int height, int width) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", "/camera_left", ros::Time(0), transform);
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    auto origin_z = transform.getOrigin().getZ();
    auto origin_y = transform.getOrigin().getY();
    auto HFOV = toRadians(66.0);
    auto VFOV = toRadians(47.6);
    pitch = -roll;

    for (vector<Point> contour : contours) {
        for (Point p : contour) {
            int xP = p.x;
            int yP = p.y + (height / 2 - 100);

            auto pitch_offset = ((float) (yP - height / 2) / height) * VFOV;
            auto y = origin_z /tan(pitch + pitch_offset) + origin_y;

            auto theta = ((float) (xP - width / 2) / width) * HFOV;
            auto x = y * tan(theta);

            cloud->points.push_back(PointXYZ(x, y, 0));
        }
    }

	cloud->header.frame_id = "base_footprint";
	return cloud;
}
