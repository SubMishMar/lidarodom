#include "utility.h"


class PcdProcessor{
private:

    ros::NodeHandle nh;

    // subscribers
    ros::Subscriber subLaserCloud;

    // publishers
    ros::Publisher pubFilteredCloud;

    // Header for input cloud
    std_msgs::Header cloudHeader;

	// libpointmatcher
	PM::DataPointsFilters inputFilters;

    //parameters
    double minOverlap;
    double maxOverlapToMerge;
    int minReadingPointCount;

public:
    PcdProcessor():
        nh("~"){

        subLaserCloud = nh.subscribe("/velodyne_points", 1, &PcdProcessor::cloudHandler, this);
        pubFilteredCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 1);
        readParams();
        allocateMemory();
        resetParameters();
    }

    void readParams() {
    	nh.param<double>("minOverlap", minOverlap, 0.5);
    	nh.param<double>("maxOverlapToMerge", maxOverlapToMerge, 0.9);
    	nh.param<int>("minReadingPointCount", minReadingPointCount, 2000);
    	std::string configFileName;
		if (ros::param::get("~inputFiltersConfig", configFileName)){
			std::ifstream ifs(configFileName.c_str());
			if (ifs.good()){
				inputFilters = PM::DataPointsFilters(ifs);
			} else {
				ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
			}
		}
    }

    void allocateMemory(){

    }

    void resetParameters(){
    }

    ~PcdProcessor(){}

    void processCloud(std::unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq) {
    	const size_t goodCount(newPointCloud->features.cols()); // Number of points in the cloud
    	if(goodCount == 0) {
    		ROS_ERROR("[ICP] No Points for doing ICP");
    	}
    	const int dimp1(newPointCloud->features.rows()); // no of dimensions, 3D: 4, 2D: 3
 	    
 	    //Adding Time Stamp
 	    if(!(newPointCloud->descriptorExists("stamps_Msec") 
 	    	&& newPointCloud->descriptorExists("stamps_sec") 
 	    	&& newPointCloud->descriptorExists("stamps_nsec"))) {
			const float Msec = round(stamp.sec/1e6);
			const float sec = round(stamp.sec - Msec*1e6);
			const float nsec = round(stamp.nsec);

			const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);
			const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
			const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
			newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
			newPointCloud->addDescriptor("stamps_sec", desc_sec);
			newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		}

		int ptsCount = newPointCloud->getNbPoints();
		if(ptsCount < minReadingPointCount) {
			ROS_ERROR_STREAM("[ICP] Not enough points in newPointCloud: only " << ptsCount << " pts.");
			return;
		}  	

		timer t;
		inputFilters.apply(*newPointCloud);
		ROS_INFO_STREAM("[ICP] Input filters took " << t.elapsed() << " [s]");
    	
    }

    void cloudHandler(const sensor_msgs::PointCloud2& cloudMsgIn){
        std::unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
    	processCloud(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "pcd_processor");
    
    PcdProcessor PP;

    ROS_INFO("\033[1;32m---->\033[0m Processing Point Cloud");

    ros::spin();
    return 0;
}
