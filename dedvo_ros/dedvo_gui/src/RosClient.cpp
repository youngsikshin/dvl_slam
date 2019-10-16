#include <iostream>
#include <dedvo_gui/RosClient.h>

using namespace std;

RosClient::RosClient(shared_ptr<ros::NodeHandle> node,
                     shared_ptr<ros::NodeHandle> privante_nh,
                     camlidar_topic_t camlidar_topic,
                     shared_ptr<cam_lidar_queue_t> camlidar_queue)
    : camlidar_queue_(camlidar_queue)
{
    camlidar_topic_ = camlidar_topic;
    cerr << "[RosClient]\t Camera topic name : " << camlidar_topic_.cam_topic << endl;
    cerr << "[RosClient]\t Velodyne topic name : " << camlidar_topic_.lidar_topic << endl;
    // ROS setting
    this->nh_ = node;

    image_sub_.subscribe(*nh_, camlidar_topic_.cam_topic, 0);
    lidar_sub_.subscribe(*nh_, camlidar_topic_.lidar_topic, 0);

//    sync_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> (image_sub_, lidar_sub_, 0);

//    sync_ = new message_filters::Synchronizer<ApproximateSyncPolicy> (ApproximateSyncPolicy(20), image_sub_, lidar_sub_);
//    sync_->registerCallback(boost::bind(&RosClient::camlidar_callback, this, _1, _2));

//    ros::Duration max_interval_duration(0.0125);
//    sync_->setMaxIntervalDuration(max_interval_duration);
    pc_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // For TUM
    it_ = new image_transport::ImageTransport(*nh_);
    sub_ = it_->subscribe(camlidar_topic_.cam_topic, 1, &RosClient::image_callback, this);
    cloud_sub_ = nh_->subscribe(camlidar_topic_.lidar_topic, 1, &RosClient::cloud_callback, this);
}

void RosClient::run()
{
    ros::spin();
}

void RosClient::image_callback(const sensor_msgs::ImageConstPtr& cam) {
//    cv_ptr_ = cv_bridge::toCvCopy(cam, sensor_msgs::image_encodings::MONO8);
//    cerr << "image_callback : " << cam->header.stamp.toNSec() << endl;

//    cv_ptr_ = cv_bridge::toCvCopy(cam, cam->encoding);

//    std::lock_guard<std::mutex> lg(camlidar_queue_->mtx_img);
    std::unique_lock<std::mutex> ul(camlidar_queue_->mtx_img);

//    sensor_msgs::Image img = *cam;

    camlidar_queue_->img_queue.push(cam);
//    camlidar_queue_->img_queue.push(cv_ptr_);

//    camlidar_queue_->cond_camlidar.notify_one();


    ul.unlock();
    camlidar_queue_->cond_img.notify_one();
}

void RosClient::cloud_callback (const sensor_msgs::PointCloud2ConstPtr& lidar)
{
//    cerr << "cloud_callback : " << lidar->header.stamp.toNSec() << endl;

//    pcl::fromROSMsg(*lidar, *pc_ptr_);

//    std::lock_guard<std::mutex> lg(camlidar_queue_->mtx_pc);
    std::unique_lock<std::mutex> ul(camlidar_queue_->mtx_pc);

//    camlidar_queue_->pc_queue.push(lidar);
//    sensor_msgs::PointCloud2 pc = *lidar;
    camlidar_queue_->pc_queue.push(lidar);


//    pc = camlidar_queue_->pc_queue.front();
//    camlidar_queue_->pc_queue.push(pc_ptr_);

    ul.unlock();
    camlidar_queue_->cond_pc.notify_one();
}


void RosClient::camlidar_callback(const sensor_msgs::ImageConstPtr& cam, const sensor_msgs::PointCloud2ConstPtr& lidar)
{
//    ros::Time tic = ros::Time::now();

//    double sensor_time_diff = static_cast<double> (cam->header.stamp.toNSec() - lidar->header.stamp.toNSec())*1e-9;
//    if (sensor_time_diff > 0.5)   return;

//    cerr << "[RosClient]\t time diff : " << sensor_time_diff << endl;

//    cerr << "[RosClient]\t camlidar_callback..." << cnt_<< endl;


//    cv_ptr_ = cv_bridge::toCvCopy(cam, cam->encoding);
////    cv_ptr_ = cv_bridge::toCvCopy(cam, sensor_msgs::image_encodings::MONO8);

//    pcl::fromROSMsg(*lidar, *pc_ptr_);

//    std::lock_guard<std::mutex> lg(camlidar_queue_->mtx_camlidar);

//        camlidar_queue_->timestamp_queue.push(lidar->header.stamp.toNSec());
//        camlidar_queue_->img_queue.push(cv_ptr_);
//        camlidar_queue_->pc_queue.push(pc_ptr_);

//    camlidar_queue_->cond_camlidar.notify_one();

//    cerr << "[RosClient]\t " << cnt_++ << " published" << endl;

//    ros::Time toc = ros::Time::now();

//    cerr << (toc-tic).toSec() << endl;
}
