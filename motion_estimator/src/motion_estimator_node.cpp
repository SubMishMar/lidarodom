#include "utility.h"


class MotionEstimator{
private:

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn_1;
    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    std_msgs::Header cloudHeader;

    bool first_time;

    tf::TransformListener listener;
    tf::StampedTransform transform;
public:
    MotionEstimator():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &MotionEstimator::cloudHandler, this);
        first_time = true;
        allocateMemory();
    }

    void allocateMemory(){
        laserCloudIn_1.reset(new pcl::PointCloud<PointType>());
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
    }

    void resetVariables(){
        laserCloudIn_1->clear();
        *laserCloudIn_1 = *laserCloudIn;
        laserCloudIn->clear();
    }

    ~MotionEstimator(){}

    void getInitTransform() {
        try{
          ros::Time now = ros::Time::now();
          listener.waitForTransform("/world", "/init_frame",
                                    now, ros::Duration(1.0));
          listener.lookupTransform("/world", "/init_frame",  
                                    now, transform);
          tf::Vector3 origin = transform.getOrigin();
          tf::Quaternion rotn = transform.getRotation();
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
    }

    void runICP() {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(laserCloudIn_1);
        icp.setInputTarget(laserCloudIn);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        std::cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){  
        cloudHeader = laserCloudMsg->header;
        if(first_time) {
            first_time = false;
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_1);
            getInitTransform();         
        } else {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
            std::cout << laserCloudIn_1->points.size() << "\t" << laserCloudIn->points.size() << std::endl;
            runICP();
            resetVariables();
        }
        
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "motion_estimator");
    
    MotionEstimator ME;

    ROS_INFO("\033[1;32m---->\033[0m Estimating Motion");

    ros::spin();
    return 0;
}
