#include "utility.h"


class PcdProcessor{
private:

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher pubFilteredCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;


    std_msgs::Header cloudHeader;
    sensor_msgs::PointCloud2 laserCloudOut;

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
        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);
    }

    void resetParameters(){
        laserCloudIn->clear();
    }

    ~PcdProcessor(){}

	void downSampleCloud() {
		//std::cout << "downSampleCloud: " << laserCloudIn->size() << "\t";
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		pcl::PCLPointCloud2::Ptr laserCloudIn2(new pcl::PCLPointCloud2 ());
		pcl::toPCLPointCloud2(*laserCloudIn, *laserCloudIn2);
		sor.setInputCloud(laserCloudIn2);
		sor.setLeafSize (0.30f, 0.30f, 0.30f);
		sor.filter(*laserCloudIn2);
		pcl::fromPCLPointCloud2(*laserCloudIn2, *laserCloudIn);
		//std::cout << laserCloudIn->size() << std::endl;
	}

	void passThroughFilter() {
		//std::cout << "passThroughFilter: " << laserCloudIn->size() << "\t";
		
		pcl::PassThrough<pcl::PointXYZ> pass_z;
		// pass_z.setNegative(true);
		pass_z.setInputCloud (laserCloudIn);
		pass_z.setFilterFieldName ("z");
		pass_z.setFilterLimits (-1, 2);
		pass_z.filter (*laserCloudIn);

		//std::cout << laserCloudIn->size() << std::endl;
	}

    void removeGround() {
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	pcl::SACSegmentation<pcl::PointXYZ> seg;
    	seg.setOptimizeCoefficients (true);
    	seg.setModelType (pcl::SACMODEL_PLANE);
    	seg.setMethodType (pcl::SAC_RANSAC);
    	seg.setDistanceThreshold (0.3);
    	seg.setInputCloud (laserCloudIn);
    	seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0) {
		   ROS_ERROR("Could not estimate a planar model for the given dataset.");
		   return;
		} else {
			//std::cout << "laserCloudIn: " << laserCloudIn->size() << "\t";
			extract.setInputCloud(laserCloudIn);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*laserCloudIn);
			//std::cout << laserCloudIn->size() << std::endl;
		}
		passThroughFilter();
	}

    void publishCloud() {
		pcl::toROSMsg(*laserCloudIn, laserCloudOut);
        laserCloudOut.header = cloudHeader;
		pubFilteredCloud.publish(laserCloudOut);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        //ROS_INFO("Received Point Cloud");
        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        downSampleCloud();
        removeGround();
        publishCloud();
        resetParameters();
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "pcd_processor");
    
    PcdProcessor PP;

    ROS_INFO("\033[1;32m---->\033[0m Processing Point Cloud");

    ros::spin();
    return 0;
}
