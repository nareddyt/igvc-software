#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace pcl;
using namespace ros;
using namespace std;

Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;
set<string> frames_seen;

void frame_callback(const PointCloud<PointXYZ>::ConstPtr &msg, const string &topic) {
    PointCloud<PointXYZ> transformed;
    tf::StampedTransform transform;

    // TODO

    _pointcloud_pub.publish(transformed);
}

int main(int argc, char** argv) {
    // FIXME
    init(argc, argv, "barreldetector");

    NodeHandle nh;

    PotholeDetector det{nh};

    tf_listener = new tf::TransformListener();
    _pointcloud_pub = nh.advertise<PointCloud<PointXYZ>>("/barrel/blackout_image", 1);

    Subscriber scan_sub = nh.subscribe("/scan/pointcloud", 1, frame_callback);
    spin();

    return 0;
}
