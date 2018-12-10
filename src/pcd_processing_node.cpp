#include "utility.h"


class PcdProcessor{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFilteredCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    pcl::PointCloud<PointType>::Ptr laserCloudOut;

    std_msgs::Header cloudHeader;


public:
    PcdProcessor():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &PcdProcessor::cloudHandler, this);

        pubFilteredCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 1);

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());

    }

    void resetParameters(){
        laserCloudIn->clear();
        laserCloudOut->clear();
    }

    ~PcdProcessor(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        copyPointCloud(laserCloudMsg);
        
        resetParameters();
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    PcdProcessor PP;

    ROS_INFO("\033[1;32m---->\033[0m Processing Point Cloud");

    ros::spin();
    return 0;
}
