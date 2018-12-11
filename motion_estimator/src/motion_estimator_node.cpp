#include "utility.h"


class MotionEstimator{
private:

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    std_msgs::Header cloudHeader;

    bool first_time;

public:
    MotionEstimator():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &MotionEstimator::cloudHandler, this);
        first_time = true;
        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
    }

    void resetParameters(){
        laserCloudIn->clear();
    }

    ~MotionEstimator(){}

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        resetParameters();
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "motion_estimator");
    
    MotionEstimator ME;

    ROS_INFO("\033[1;32m---->\033[0m Estimating Motion");

    ros::spin();
    return 0;
}
