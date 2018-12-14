#include "utility.h"


class MotionEstimator{
private:

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher pubICPPose;
    ros::Publisher pubICPPath;

    pcl::PointCloud<PointT>::Ptr laserCloudIn_1;
    pcl::PointCloud<PointT>::Ptr laserCloudIn;

    std_msgs::Header cloudHeader;

    bool first_time;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    Eigen::Matrix4d global_init;
    Eigen::Matrix4f global_transformation;
    
    geometry_msgs::PoseStamped output_pose;
    nav_msgs::Path output_path;

public:
    MotionEstimator():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &MotionEstimator::cloudHandler, this);
        pubICPPose = nh.advertise<geometry_msgs::PoseStamped>("/icp/pose", 10);
        pubICPPath = nh.advertise<nav_msgs::Path>("/icp/path", 10);
        first_time = true;
        global_transformation = Eigen::Matrix4f::Identity();
        allocateMemory();
    }

    void allocateMemory(){
        laserCloudIn_1.reset(new pcl::PointCloud<PointT>());
        laserCloudIn.reset(new pcl::PointCloud<PointT>());
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
          global_transformation = global_init.cast<float>();
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
    }

    void publishICPPath() {
        output_path.header = cloudHeader;
        output_path.header.frame_id = "world";
        output_path.poses.push_back(output_pose);
        pubICPPath.publish(output_path);
    }

    void publishICPPose() {
        tf::Vector3 origin;
        origin.setValue(static_cast<double>(global_transformation(0,3)),
                        static_cast<double>(global_transformation(1,3)),
                        static_cast<double>(global_transformation(2,3)));
        Eigen::Matrix3f rotn_float = global_transformation.block(0, 0, 3, 3).cast<float>();
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
        pubICPPose.publish(output_pose);
        publishICPPath();
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
            global_transformation = global_transformation*transformation;
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

    void runICPPointToPlane() {
        // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
        // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
        // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
        // addNormal(laserCloudIn_1, cloud_source_normals);
        // addNormal(laserCloudIn, cloud_target_normals);
        // pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
        // icp->setInputSource(cloud_source_normals);
        // icp->setInputTarget(cloud_target_normals);
        // icp->align(*Final);
        // if(icp->hasConverged()) {
        //     Eigen::Matrix4f transformation = icp->getFinalTransformation().inverse();
        //     global_transformation = global_transformation*transformation;
        //     publishICPPose();
        // } else {
        //    ROS_WARN("ICP didn't converge"); 
        // }
        PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch (30);

        norm_est.setInputCloud (laserCloudIn_1);
        norm_est.compute (*points_with_normals_src);
        pcl::copyPointCloud (*laserCloudIn_1, *points_with_normals_src);

        norm_est.setInputCloud (laserCloudIn);
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*laserCloudIn, *points_with_normals_tgt);

    }
    
    //https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
    //http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){  
        cloudHeader = laserCloudMsg->header;
        if(first_time) {
            first_time = false;
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_1);
            getInitTransform();
            publishICPPose();          
        } else {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
            //runICPPointToPoint();
            runICPPointToPlane();
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
