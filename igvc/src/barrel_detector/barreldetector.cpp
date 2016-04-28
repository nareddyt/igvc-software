#include "barreldetector.h"

void BarrelDetector::img_callback(const sensor_msgs::ImageConstPtr& msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "");
    model.fromCameraInfo(cam_info);

    // FIXME change to use actual image width and height instead of 0, 720, 1280
    int min_x = 1280;
    int max_x = 0;
    int min_y = 720;
    int max_y = 0;
    pclMutex.lock();
    for (pcl::PointXYZ p: *cloud) {

        // FIXME this transform is off. Mistii.urdf has already been edited to work with this.
        // Could be due to negative y cordinates in the base_frame pointcloud. So fix this first (in other method)
        cerr<<"Base Frame: " + to_string(p.x) + " " + to_string(p.y) + " " + to_string(p.z)<<endl;
        tf::Point tfp = cameraFrameFromBaseFrame * tf::Point(p.x, p.y, p.z);
        cerr<<"Camera Frame: " + to_string(tfp.x()) + " " + to_string(tfp.y()) + " " + to_string(tfp.z())<<endl;
        cv::Point2d point = model.project3dToPixel(cv::Point3d(tfp.x(), tfp.y(), tfp.z()));
        cerr<<"Image Pixel: " + to_string(point.x) + " " + to_string(point.y)<<endl;

        // FIXME change to use actual image width and height instead of 0, 720, 1280
        if (point.x < min_x && point.x >= 0) {
            min_x = point.x;
        }
        if (point.x > max_x && point.x <= 1280) {
            max_x = point.x;
        }
        if (point.y < min_y && point.y >= 0) {
            min_y = point.y;
        }
        if (point.y > max_y && point.y <= 720) {
            max_y = point.y;
        }
        cerr<<"---"<<endl;
    }
    cloud->clear();
    pclMutex.unlock();

    if (min_x <= max_x && min_y <= max_y) {
        cerr<<"Drew Rectangle: min_x = " + to_string(min_x) + ", max_x = " + to_string(max_x)<<endl;
        cv::rectangle(cv_ptr->image, cv::Point(min_x, min_y), cv::Point(max_x, max_y),
            cv::Scalar(255, 0, 0), CV_FILLED);
    }

    cerr<<"================="<<endl;

    _barrel_filt_img.publish(cv_ptr->toImageMsg());
}

void BarrelDetector::pcl_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
    pclMutex.lock();
    cloud->clear();

    for (pcl::PointXYZ p: *msg) {
        cerr<<"Lidar Frame: " + to_string(p.x) + " " + to_string(p.y) + " " + to_string(p.z)<<endl;
        tf::Point tfp = baseFrameFromLidarFrame * tf::Point(p.x, p.y, p.z);

        // FIXME we are getting negative y cordinates after the transform! This should never happen
        cerr<<"Base Frame: " + to_string(tfp.x()) + " " + to_string(tfp.y()) + " " + to_string(tfp.z())<<endl;
        cloud->points.push_back(pcl::PointXYZ(tfp.x(), tfp.y(), 0.0));
        cloud->points.push_back(pcl::PointXYZ(tfp.x(), tfp.y(), 1.5));
        cerr<<"---"<<endl;
    }
    pclMutex.unlock();
    cerr<<"================="<<endl;
}

BarrelDetector::BarrelDetector(ros::NodeHandle &handle)
    : _it(handle),
      tf_listener(handle) {
    _src_img = _it.subscribeCamera("/stereo/left/image_raw", 1, &BarrelDetector::img_callback, this);
    _lidar_pointcloud = handle.subscribe("/scan/pointcloud", 1, &BarrelDetector::pcl_callback, this);
    _barrel_filt_img = _it.advertise("/barrel_filt_img", 1);
    cloud = new pcl::PointCloud<pcl::PointXYZ>;

    tf_listener.waitForTransform("/base_footprint", "/lidar", ros::Time(0), ros::Duration(60));
    tf_listener.lookupTransform("/base_footprint", "/lidar", ros::Time(0), baseFrameFromLidarFrame);

    tf_listener.waitForTransform("/camera_left", "/base_footprint", ros::Time(0), ros::Duration(60));
    tf_listener.lookupTransform("/camera_left", "/base_footprint", ros::Time(0), cameraFrameFromBaseFrame);
}
