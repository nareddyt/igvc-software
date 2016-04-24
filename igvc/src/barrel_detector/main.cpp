#include "barreldetector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "barreldetector");

    ros::NodeHandle nh;

    BarrelDetector det{nh};

    ros::spin();

    return 0;
}
