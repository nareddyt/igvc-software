#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>

using namespace pcl;
using namespace ros;
using namespace std;
using namespace tf;

Publisher _pointcloud_pub;
set<string> frames_seen;

void frame_callback(const PointCloud<PointXYZ>::ConstPtr &msg, const string &topic) {
    PointCloud<PointXYZ> transformed;
    tf::StampedTransform transform;
    if(frames_seen.find(msg->header.frame_id) == frames_seen.end()) {
        frames_seen.insert(msg->header.frame_id);
        tf_listener->waitForTransform("/map", msg->header.frame_id, Time(0), Duration(5));
    }

    tf_listener->lookupTransform("/map", msg->header.frame_id, Time(0), transform);
    pcl_ros::transformPointCloud(*msg, transformed, transform);
    for (auto point : transformed) {
        point.z = 0;
    }

    frames[topic] = transformed;

    publish_map();

    _pointcloud_pub.publish(*cloud_for_pub);
}

int main(int argc, char** argv) {
    init(argc, argv, "scan_to_pointcloud");

    NodeHandle nh;

    _pointcloud_pub = nh.advertise<PointCloud<PointXYZ> >("/barrel/blackout_image", 1);

    Subscriber scan_sub = nh.subscribe("/scan/pointcloud", 1, frame_callback);

    spin();
}
