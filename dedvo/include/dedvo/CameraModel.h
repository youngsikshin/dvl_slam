#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "Point.h"

using namespace std;

namespace dedvo {

class CameraModel
{
public:
    typedef shared_ptr<CameraModel> Ptr;

    CameraModel() {};
    CameraModel(int width, int height) : width_(width), height_(height)
    {

    };

    virtual ~CameraModel() {};

    virtual bool is_in_image(const Eigen::Vector3f& point, int boundary) = 0;
    virtual bool is_in_image(const POINT& point, int boundary) = 0;
    virtual bool is_in_image(const int u, const int v, int boundary) = 0;
    virtual bool is_in_image(Eigen::Vector2f uv, int boundary) = 0;
    virtual inline bool is_in_image(Eigen::Vector2f uv, int boundary, float scale) = 0;

    // virtual Eigen::Vector3f uv_to_xyz(const Eigen::Vector2f& uv);
    virtual inline Eigen::Vector2f xyz_to_uv(const Eigen::Vector3f& xyz) = 0;
    virtual inline Eigen::Vector2f xyz_to_uv(const POINT& xyz) = 0;
//    virtual Eigen::Vector2f xyz_to_uv_sse(const POINT& xyz) = 0;
    virtual inline vector<Eigen::Vector2f> pointcloud_to_uv(const PointCloud& pc, float scale) = 0;

    virtual void undistort_image(const cv::Mat& raw, cv::Mat& rectified) = 0;

    inline int width() { return width_; }
    inline int height() { return height_; }

protected:
    int width_;
    int height_;
};

}   // dedvo
