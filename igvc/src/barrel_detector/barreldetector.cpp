#include "barreldetector.h"

void BarrelDetector::img_callback(const sensor_msgs::ImageConstPtr& msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "");
    model.fromCameraInfo(cam_info);
    int min_x = 1280;
    int max_x = 0;
    int min_y = 720;
    int max_y = 0;
    pclMutex.lock();
    for (pcl::PointXYZ p: *cloud) {
        tf::Point tfp = cameraFrameFromBaseFrame * tf::Point(p.x, p.y, p.z);
        cv::Point2d point = model.project3dToPixel(cv::Point3d(tfp.x(), tfp.y(), tfp.z()));
        if (point.x < min_x) {
            min_x = point.x;
        } 
        if (point.x > max_x) {
            max_x = point.x;
        }
        if (point.y < min_y) {
            min_y = point.y;
        }
        if (point.y > max_y) {
            max_y = point.y;
        }
    }
    pclMutex.unlock();
    cv::rectangle(cv_ptr->image, cv::Point(min_x, min_y), cv::Point(max_x, max_y),
            cv::Scalar(255, 0, 0), CV_FILLED);
    _barrel_filt_img.publish(cv_ptr->toImageMsg());
}

void BarrelDetector::pcl_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
    pclMutex.lock();
    for (pcl::PointXYZ p: *msg) {
        tf::Point tfp = baseFrameFromLidarFrame * tf::Point(p.x, p.y, p.z);
        cloud->points.push_back(pcl::PointXYZ(tfp.x(), tfp.y(), 0.0));
        cloud->points.push_back(pcl::PointXYZ(tfp.x(), tfp.y(), 1.5));
    }
    pclMutex.unlock();
}

BarrelDetector::BarrelDetector(ros::NodeHandle &handle)
    : _it(handle),
      tf_listener(handle) {
    _src_img = _it.subscribeCamera("/left/image_rect_color", 1, &BarrelDetector::img_callback, this);
    _lidar_pointcloud = handle.subscribe("/scan/pointcloud", 1, &BarrelDetector::pcl_callback, this);
    _barrel_filt_img = _it.advertise("/barrel_filt_img", 1);
    tf_listener.waitForTransform("/base_footprint", "/lidar", ros::Time(0), ros::Duration(60));
    tf_listener.waitForTransform("/camera_left", "/base_footprint", ros::Time(0), ros::Duration(60));
    tf_listener.lookupTransform("/base_footprint", "/lidar", ros::Time(0), baseFrameFromLidarFrame);
    tf_listener.lookupTransform("/camera_left", "/base_footprint", ros::Time(0), cameraFrameFromBaseFrame);
    cloud = new pcl::PointCloud<pcl::PointXYZ>;
}
