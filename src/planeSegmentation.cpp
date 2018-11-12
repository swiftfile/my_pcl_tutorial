//
// Created by file on 18/10/30.
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

static int RS_ROWS = 640;
static int RS_COLS = 480;

void plane_cb(const sensor_msgs::PointCloud2ConstPtr &sub_cloud) {
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //cloud(pcl::Pointcloud)
    pcl::fromROSMsg(*sub_cloud, cloud);


    for (size_t j = 0; j < cloud.points.size(); ++j) {
        cloud.points[j].a = 255;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //create the segmentation object

    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

    //Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    double dist_th;
    ros::param::get("dist_th", dist_th);
    seg.setDistanceThreshold(dist_th);

    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);

    ROS_INFO("plane equation is %f x + %f y + %f z = 0",coefficients->values[0],coefficients->values[1],coefficients->values[2]);




//以下は平面予測のぷよ
    //Fill in the cloud data

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    } else {
        for (size_t i = 0; i < inliers->indices.size(); ++i) {
            cloud.points[inliers->indices[i]].r = 255;
            cloud.points[inliers->indices[i]].g = 0;
            cloud.points[inliers->indices[i]].b = 0;
            cloud.points[inliers->indices[i]].a = 255;
        };


        pub.publish(cloud);
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "planeSegmentation");
    ros::NodeHandle nh;
    ROS_INFO("planeSegmentation has launched.");
    ros::param::set("dist_th", 0.1);
//    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, plane_cb);
    ros::Subscriber sub = nh.subscribe("input", 1, plane_cb);

    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("/planedcloud", 1);

    ros::spin();


}