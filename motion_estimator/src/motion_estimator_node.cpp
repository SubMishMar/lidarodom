#include "utility.h"


class MotionEstimator{
private:

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    //ros::Subscriber subIMU;
    ros::Publisher pubICPPose;
    ros::Publisher pubKFPose;
    ros::Publisher pubICPPath;
    ros::Publisher pubKFPath;
    //ros::Publisher pubCloud;

    pcl::PointCloud<PointT>::Ptr laserCloudIn_1;
    pcl::PointCloud<PointT>::Ptr laserCloudIn;
    pcl::PointCloud<PointT>::Ptr laserCloudIn_lastKeyFrame;

    std_msgs::Header cloudHeader;
    std_msgs::Header imuHeader;

    bool first_time;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    Eigen::Matrix4d global_init;
    Eigen::Matrix4f global_pose;
    Eigen::Matrix4f last_keyframe_pose;
    Eigen::Matrix4f global_pose_KF;

    geometry_msgs::PoseStamped output_pose;
    nav_msgs::Path output_path;

    geometry_msgs::PoseStamped output_KFpose;
    nav_msgs::Path output_KFpath;

    //sensor_msgs::PointCloud2 laserCloudOut;

public:
    MotionEstimator():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &MotionEstimator::cloudHandler, this);
        //subIMU = nh.subscribe<sensor_msgs::Imu>("/imu_data",10, &MotionEstimator::imuHandler, this);
        //pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/icp/cloud", 1);
        pubKFPose = nh.advertise<geometry_msgs::PoseStamped>("/icp/kf/pose", 10);
        pubKFPath = nh.advertise<nav_msgs::Path>("/icp/kf/path", 10);
        
        pubICPPose = nh.advertise<geometry_msgs::PoseStamped>("/icp/pose", 10);
        pubICPPath = nh.advertise<nav_msgs::Path>("/icp/path", 10);
        first_time = true;
        global_pose_KF = global_pose = last_keyframe_pose = Eigen::Matrix4f::Identity(); 
        allocateMemory();
    }

    void allocateMemory(){
        laserCloudIn_1.reset(new pcl::PointCloud<PointT>());
        laserCloudIn.reset(new pcl::PointCloud<PointT>());
        laserCloudIn_lastKeyFrame.reset(new pcl::PointCloud<PointT>());
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
          tf::Transform tf_init;
          tf_init.setOrigin(transform.getOrigin());
          tf_init.setRotation(transform.getRotation());
          Eigen::Affine3d eig_init;
          tf::transformTFToEigen(tf_init, eig_init);
          Eigen::Matrix4d global_init = eig_init.matrix();
          global_pose = last_keyframe_pose = global_init.cast<float>();
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
    }

    void publishTF() {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z) );
        transform.setRotation(tf::Quaternion(output_pose.pose.orientation.x, output_pose.pose.orientation.y, output_pose.pose.orientation.z, output_pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));       
    }

    void publishICPPath() {
        output_path.header = cloudHeader;
        output_path.header.frame_id = "world";
        output_path.poses.push_back(output_pose);
        pubICPPath.publish(output_path);
    }

    void publishKFPath() {
        output_KFpath.header = cloudHeader;
        output_KFpath.header.frame_id = "world";
        output_KFpath.poses.push_back(output_KFpose);
        pubKFPath.publish(output_KFpath);
    }

    void publishICPPose() {
        tf::Vector3 origin;
        origin.setValue(static_cast<double>(global_pose(0,3)),
                        static_cast<double>(global_pose(1,3)),
                        static_cast<double>(global_pose(2,3)));
        Eigen::Matrix3f rotn_float = global_pose.block(0, 0, 3, 3).cast<float>();
        tf::Matrix3x3 rotn_tf;
        tf::matrixEigenToTF(rotn_float.cast<double>(), rotn_tf);
        tf::Quaternion tfqt;
        rotn_tf.getRotation(tfqt);
        output_pose.header = cloudHeader;
        output_pose.header.frame_id = "world";
        output_pose.pose.position.x = origin.getX();
        output_pose.pose.position.y = origin.getY();
        output_pose.pose.position.z = origin.getZ();
        output_pose.pose.orientation.x = tfqt[0];
        output_pose.pose.orientation.y = tfqt[1];
        output_pose.pose.orientation.z = tfqt[2];
        output_pose.pose.orientation.w = tfqt[3];
        publishTF();
        //publishCloud();
        pubICPPose.publish(output_pose);
        publishICPPath();
    }

    void publishKFPose() {
        tf::Vector3 origin;
        origin.setValue(static_cast<double>(global_pose_KF(0,3)),
                        static_cast<double>(global_pose_KF(1,3)),
                        static_cast<double>(global_pose_KF(2,3)));
        Eigen::Matrix3f rotn_float = global_pose_KF.block(0, 0, 3, 3).cast<float>();
        tf::Matrix3x3 rotn_tf;
        tf::matrixEigenToTF(rotn_float.cast<double>(), rotn_tf);
        tf::Quaternion tfqt;
        rotn_tf.getRotation(tfqt);
        output_KFpose.header = cloudHeader;
        output_KFpose.header.frame_id = "world";
        output_KFpose.pose.position.x = origin.getX();
        output_KFpose.pose.position.y = origin.getY();
        output_KFpose.pose.position.z = origin.getZ();
        output_KFpose.pose.orientation.x = tfqt[0];
        output_KFpose.pose.orientation.y = tfqt[1];
        output_KFpose.pose.orientation.z = tfqt[2];
        output_KFpose.pose.orientation.w = tfqt[3];
        pubKFPose.publish(output_KFpose);
        publishKFPath();
    }

    void runICPPointToPoint() {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(laserCloudIn_1);
        icp.setInputTarget(laserCloudIn);
        icp.setMaxCorrespondenceDistance (0.10);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        if(icp.hasConverged()) {
            Eigen::Matrix4f transformation = (icp.getFinalTransformation()).inverse();
            global_pose = global_pose*transformation;
            publishICPPose();
        } else {
            ROS_WARN("ICP didn't converge");
        }
    }

    void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
    ){
       pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

       pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
       searchTree->setInputCloud ( cloud );

       pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
       normalEstimator.setInputCloud ( cloud );
       normalEstimator.setSearchMethod ( searchTree );
       normalEstimator.setKSearch ( 15 );
       normalEstimator.compute ( *normals );
      
       pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
    }

    void pairAlign(pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt, Eigen::Matrix4f &transformation) {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
        addNormal(cloud_src, cloud_source_normals);
        addNormal(cloud_tgt, cloud_target_normals);
        pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
        icp->setInputSource(cloud_source_normals);
        icp->setInputTarget(cloud_target_normals);

        //icp->setMaxCorrespondenceDistance(0.50);
        //icp->setMaximumIterations (50);
        icp->setTransformationEpsilon(1e-8);
        //icp->setEuclideanFitnessEpsilon (1);

        icp->align(*Final);
        if(icp->hasConverged()) {
            //std::cout << " score: " << icp->getFitnessScore() << std::endl;
            transformation = icp->getFinalTransformation().inverse();

        } else {
           ROS_WARN("ICP didn't converge"); 
        }
    }

    void runICPPointToPlane() {
        Eigen::Matrix4f transformation_ij;
        pairAlign(laserCloudIn_1, laserCloudIn, transformation_ij);
        global_pose = global_pose*transformation_ij;
        publishICPPose();
        Eigen::Matrix4f poseDiff = last_keyframe_pose.inverse()*global_pose;
        Eigen::Vector3f positionDiff{poseDiff(0,3), poseDiff(1,3), poseDiff(2,3)};
        if(positionDiff.norm() >= KF_THRESHOLD) {
            Eigen::Matrix4f transformation_KFj;
            pairAlign(laserCloudIn_lastKeyFrame, laserCloudIn, transformation_KFj);
            global_pose_KF = last_keyframe_pose*transformation_KFj;
            publishKFPose();
            *laserCloudIn_lastKeyFrame = *laserCloudIn;
            last_keyframe_pose = global_pose;
        }
    }
    
    //https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
    //http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){  
        cloudHeader = laserCloudMsg->header;
        if(first_time) {
            first_time = false;
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_1);
            *laserCloudIn_lastKeyFrame = *laserCloudIn_1;
            getInitTransform();
            publishICPPose();         
        } else {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
            //runICPPointToPoint();
            runICPPointToPlane();
            resetVariables();
        }
        
    }

    // void processIMU(const sensor_msgs::ImuConstPtr &imu_msg) {
    //     // TODO: Continue working on this la'er
    //     imuHeader = imu_msg->header;

    //     // Acceleration
    //     double ddx = imu_msg->linear_acceleration.x;
    //     double ddy = imu_msg->linear_acceleration.y;
    //     double ddz = imu_msg->linear_acceleration.z - gravity;
    //     Eigen::Vector3d linear_acceleration{ddx, ddy, ddz};
    //     std::cout << ddz << std::endl;
    //     // Angular Velocity
    //     double rx = imu_msg->angular_velocity.x;
    //     double ry = imu_msg->angular_velocity.y;
    //     double rz = imu_msg->angular_velocity.z;
    //     Eigen::Vector3d angular_velocity{rx, ry, rz};   

    // }

    // void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg) {
    //     processIMU(imu_msg);
    // }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "motion_estimator");
    
    MotionEstimator ME;

    ROS_INFO("\033[1;32m---->\033[0m Estimating Motion");

    ros::spin();
    return 0;
}
