#ifndef BARRELDETECTOR_H
#define BARRELDETECTOR_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <mutex>

using namespace std;
using namespace cv;
using namespace pcl;

class BarrelDetector {
public:
    BarrelDetector(ros::NodeHandle &handle);

private:
    void img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
    void pcl_callback(const PointCloud<PointXYZ>::ConstPtr& msg);

    PointCloud<pcl::PointXYZ> *cloud;
    tf::StampedTransform cameraFrameFromBaseFrame;
    tf::StampedTransform baseFrameFromLidarFrame;
    image_geometry::PinholeCameraModel model;
    Mutex pclMutex;

    // ROS COMMUNICATION
    image_transport::ImageTransport _it;
    image_transport::Publisher _barrel_filt_img;
    image_transport::CameraSubscriber _src_img;
    ros::Subscriber _lidar_pointcloud;
    tf::TransformListener tf_listener;
};
#endif // BARRELDETECTOR_H
