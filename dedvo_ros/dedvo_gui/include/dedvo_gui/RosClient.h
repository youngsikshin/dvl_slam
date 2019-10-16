#pragma once

#include <memory>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "Resources.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> ApproximateSyncPolicy;

class RosClient
{
public:
//    RosClient(ros::NodeHandle node,
//              ros::NodeHandle privante_nh,
//              camlidar_topic_t camlidar_topic,
//              shared_ptr<cam_lidar_queue_t> camlidar_queue);
    RosClient(shared_ptr<ros::NodeHandle> node,
              shared_ptr<ros::NodeHandle> privante_nh,
              camlidar_topic_t camlidar_topic,
              shared_ptr<cam_lidar_queue_t> camlidar_queue);
    ~RosClient() { }

    void run();

private:
    camlidar_topic_t camlidar_topic_;
    shared_ptr<ros::NodeHandle> nh_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;

    message_filters::Synchronizer<ApproximateSyncPolicy> *sync_;
//     message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> *sync_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber sub_;
    ros::Subscriber cloud_sub_;

    shared_ptr<cam_lidar_queue_t> camlidar_queue_;

    cv_bridge::CvImagePtr cv_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr_;

    int cnt_ = 0;

    // callback function
    void image_callback(const sensor_msgs::ImageConstPtr& cam);
    void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& lidar);
    void camlidar_callback(const sensor_msgs::ImageConstPtr& cam, const sensor_msgs::PointCloud2ConstPtr& lidar);
};
