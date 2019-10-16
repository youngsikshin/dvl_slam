#pragma once

#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <DBoW2/DBoW2.h>
#include "Datatypes.h"
#include "Config.h"
#include "Point.h"
#include "CameraModel.h"
#include "ORBExtractor.h"
#include "ORBVocabulary.h"
#include <mutex>

using namespace std;

namespace dedvo
{

class Frame
{
public:
    typedef shared_ptr<Frame> Ptr;

    Frame(const cv::Mat& img);
    Frame(const int64_t timestamp, const cv::Mat& img, PointCloud& pointcloud, CameraModel::Ptr camera);
    ~Frame();

    cv::Mat& original_img() { return original_img_; }
    cv::Mat& level(size_t idx);
//    vector<Point, Eigen::aligned_allocator<Eigen::Vector3f>>& pointcloud();
    PointCloud& pointcloud();
    CameraModel::Ptr camera() { return camera_; }

    Sophus::SE3f& Twc() { return Twc_; }
    Sophus::SE3d dTwc() { return Twc_.cast<double>(); }
    void Twc(Sophus::SE3f Twc);
    
    int get_pointcloud_size() { return pointcloud_.size(); }
    
    void show_pointcloud();
    void show_image_with_points(cv::Mat& img, size_t num_level);
    void save_image_with_points(size_t num_level, int id);


    inline static void jacobian_xyz2uv(const Eigen::Vector3f& xyz_in_f, Matrix2x6& J)
    {
        const float x = xyz_in_f[0];
        const float y = xyz_in_f[1];
        const float z_inv = 1./xyz_in_f[2];
        const float z_inv_2 = z_inv*z_inv;

        J(0,0) = -z_inv;              // -1/z
        J(0,1) = 0.0;                 // 0
        J(0,2) = x*z_inv_2;           // x/z^2
        J(0,3) = y*J(0,2);            // x*y/z^2
        J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
        J(0,5) = y*z_inv;             // y/z

        J(1,0) = 0.0;                 // 0
        J(1,1) = -z_inv;              // -1/z
        J(1,2) = y*z_inv_2;           // y/z^2
        J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
        J(1,4) = -J(0,3);             // -x*y/z^2
        J(1,5) = -x*z_inv;            // x/z
    }

private:
    void initialize_img(const cv::Mat img);
    void initialize_pc();
    void initialize_pc_OMP();

    int64_t utime_;

    CameraModel::Ptr camera_;

    cv::Mat original_img_;
    ImgPyramid img_pyramid_;
    PointCloud pointcloud_;

    int num_levels_;
    int max_level_;
    
    Sophus::SE3f Twc_;
    mutex Twc_mutex_;

};

/// Creates an image pyramid of half-sampled images.
void create_image_pyramid(const cv::Mat& img_level_0, int n_levels, ImgPyramid& pyr);

class FrameDB
{
public:
    typedef shared_ptr<FrameDB> Ptr;

    FrameDB() {}
    ~FrameDB() {}

    void add(Frame::Ptr frame) {
        unique_lock<mutex> ul{DB_mutex_};
        frameDB_.push_back(frame);
        ul.unlock();
    }

    vector<Frame::Ptr>::iterator begin() { return frameDB_.begin(); }
    vector<Frame::Ptr>::iterator end() { return frameDB_.end(); }
    size_t size() { return frameDB_.size(); }
    vector<Frame::Ptr>& frameDB() { return frameDB_; }
private:
    vector<Frame::Ptr> frameDB_;
    mutex DB_mutex_;
};

}
