#include "orne_box_tilted_lidar/convert_pointcloud.h"

convert_pointcloud::convert_pointcloud(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &convert_pointcloud::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void convert_pointcloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

