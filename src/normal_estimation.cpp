//
// Created by file on 18/11/09.
//

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;


void plane_cb(const sensor_msgs::PointCloud2ConstPtr &sub_cloud) {
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //cloud(pcl::Pointcloud)
    pcl::fromROSMsg(*sub_cloud, cloud);


    pub.publish(cloud);

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "normal_estimation");
    ros::NodeHandle nh;

    ros::param::set("dist_th", 0.1);
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, plane_cb);
//    ros::Subscriber sub = nh.subscribe("/cloud", 1, plane_cb);

    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("/planedcloud", 1);

    ros::spin();


}