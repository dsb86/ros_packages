//pcd_snapshot.cpp
// example of saving a kinect snapshot to a pcd file
// need to connect "Kinect" and start it with: roslaunch freenect_launch freenect.launch

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/ros/conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>


#include <stdlib.h>
#include <math.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>


//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs


using namespace std;


extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_original(new pcl::PointCloud<pcl::PointXYZRGB>);
const int Maxerr = 40;

bool got_kinect_image = false; //snapshot indicator
void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_image) { // once only, to keep the data stable
        ROS_INFO("got new selected kinect image");
        pcl::fromROSMsg(*cloud, *pcl_original);
        //ROS_INFO("image has  %d * %d points", pcl_original->width, pcl_original->height);
        got_kinect_image = true;
    }
}


bool find_can_by_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud) {
    int color_given[3]= {85, 20, 20};
    int color_err_RGB[3]= {0, 0, 0};

    int npts = in_cloud->points.size();
    int count=0;
    Eigen::Vector3f pt;
    
    double color_err;
    color_err = 255;
    

    for (int i = 0; i < npts; i++) 
    {
        pt = in_cloud->points[i].getVector3fMap();
        color_err_RGB[0] = abs(color_given[0] - in_cloud->points[i].r);
        color_err_RGB[1] = abs(color_given[1] - in_cloud->points[i].g);
        color_err_RGB[2] = abs(color_given[2] - in_cloud->points[i].b);

        // ROS_INFO("%f", pt[2]);

            if (color_err_RGB[0] < Maxerr && color_err_RGB[1] < Maxerr && color_err_RGB[2] < Maxerr) 
            {
                count++;

            }
            // index.push_back(i);
    }
    if (count < 250) 
    {
        ROS_INFO("Recieved %d", count);
        return false;
    }
    ROS_INFO("Recieved %d", count);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_can_find"); //node name
    ros::NodeHandle nh;


    ros::Subscriber pointcloud_subscriber = nh.subscribe("/camera/depth_registered/points", 1, kinectCB);


 //pointer for color version of pointcloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 

    //spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (pcl_original);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*pcl_filtered);

    bool can=find_can_by_color(pcl_original);

    if(can){
        ROS_INFO("We Have A Can!");
    }
    else{
        ROS_INFO("No Can");
    }

    return 0;
}
