#pragma once
#include <ros/ros.h>
#include <ros/package.h>

#include <thread>
#include <boost/lockfree/queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>

#include <QObject>
#include <QThread>
#include <QMutex>

#include <dedvo/Datatypes.h>
#include <dedvo/Point.h>
#include <dedvo/CameraModel.h>
#include <dedvo/System.h>
#include <dedvo/Frame.h>
#include <dedvo/KeyframeDB.h>

#include "Resources.h"
#include "RosClient.h"

using namespace std;

class CamLidarProcess : public QThread
{
    Q_OBJECT

public:
    typedef pair<sensor_msgs::ImageConstPtr, sensor_msgs::PointCloud2ConstPtr> CamlidarPair;

    CamLidarProcess(QObject *parent, QMutex *th_mutex,
                    shared_ptr<ros::NodeHandle> node,
                    shared_ptr<ros::NodeHandle> privante_nh);
    ~CamLidarProcess();

    QMutex *mutex;


    void status(bool running) { status_ = running; }
    bool& status()  { return status_;}
    void stop() { status_ = false; }

    void prepare_cam_lidar();
    void track_camlidar();
    void run();

    dedvo::KeyframeDB::Ptr keyframeDB() { return keyframeDB_; }
    dedvo::FrameDB::Ptr frameDB() { return frameDB_; }

signals:
    void callback_KF();

private:
    shared_ptr<cam_lidar_queue_t> camlidar_queue_;

    RosClient* ros_client_;
    std::thread *ros_client_thread_;
    std::thread *camlidar_process_thread_;
    std::thread *system_thread_;

    // Data
//    uint32_t timestamp_;
    sensor_msgs::ImageConstPtr img_ptr_;
    sensor_msgs::PointCloud2ConstPtr pc_ptr_;

    queue<CamlidarPair> camlidar_pair_queue_;
    std::mutex mtx_camlidar_;
    std::condition_variable cond_camlidar_;

    cv_bridge::CvImagePtr img_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_;

    // System
    dedvo::System::Ptr dedvo_system_;
    dedvo::CameraModel::Ptr camera_;
    dedvo::Frame::Ptr reference_, current_;

    dedvo::KeyframeDB::Ptr keyframeDB_;
    dedvo::FrameDB::Ptr frameDB_;

    dedvo::Matrix3x4 extrinsics_;

    shared_ptr<dedvo::ORBVocabulary> vocabulary_;

    bool status_;

    int cnt_ = 0;
};
