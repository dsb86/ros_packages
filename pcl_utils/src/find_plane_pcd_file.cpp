//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
// illustrates use of PCL methods: computePointNormal(), transformPointCloud(), 
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile() 
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn March 2016

#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>


#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>


#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#

using namespace std;
extern PclUtils *g_pcl_utils_ptr; 

//this fnc is defined in a separate module, find_indices_of_plane_from_patch.cpp
extern void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZ>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZ>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to hold filtered Kinect image

    vector<int> indices;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    pcl::PCDReader reader;
    reader.read ("coke_can.pcd", *pclKinect_clr_ptr);
    std::cout << "PointCloud before filtering has: " << pclKinect_clr_ptr->points.size () << " data points." << std::endl; //*

    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    //      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (pclKinect_clr_ptr);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*downsampled_kinect_ptr);
    std::cout << "PointCloud after filtering has: " << downsampled_kinect_ptr->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (downsampled_kinect_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (downsampled_kinect_ptr);
    ec.extract (cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it = (cluster_indices.begin() + 3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (downsampled_kinect_ptr->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //PointCloud<PointXYZ> cloud_xyz;
    //copyPointCloud(cloud_xyz, cloud_cluster);

    sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too

    int go=1;
    while (ros::ok()) {
       if(go==1){
            find_indices_of_plane_from_patch(downsampled_kinect_ptr, cloud_cluster, indices);
            pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud
            //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
            pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud); //convert to ros message for publication and display
        
        pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
        pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
            go=0;
        }
    }

    return 0;
}
