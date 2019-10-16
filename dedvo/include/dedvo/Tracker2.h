#pragma once
#include <iostream>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <dedvo/Config.h>
#include <dedvo/Datatypes.h>
#include <dedvo/CameraModel.h>
#include <dedvo/PinholeModel.h>
#include <dedvo/Keyframe.h>
#include <dedvo/Frame.h>
#include <dedvo/WeightFunction.h>
#include <dedvo/LSQNonlinear.hpp>

using namespace std;

namespace dedvo
{
class WeightFunction;

class Tracker2  : public LSQNonlinearGaussNewton <6, Sophus::SE3f>  //LSQNonlinearGaussNewton <6, Sophus::SE3f> LSQNonlinearLevenbergMarquardt <6, Sophus::SE3f>
{
    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2*patch_halfsize_;
    static const int patch_area_ = patch_size_*patch_size_;

    static const int pattern_length_ = 8;
    int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef shared_ptr<Tracker2> Ptr;

    Tracker2();
    ~Tracker2();

    bool tracking(Keyframe::Ptr reference, Frame::Ptr current, Sophus::SE3f& transformation);

private:
    int current_level_;

    int min_level_;
    int max_level_;

    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    Sophus::SE3f Tji_;

    Keyframe::Ptr reference_;
    Frame::Ptr current_;

    bool is_precomputed_;
    cv::Mat ref_patch_buf_, cur_patch_buf_;
    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf_;
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf_;

    vector<float> errors_;
    vector<Vector6> J_;
    vector<float> weight_;

    float affine_a_;
    float affine_b_;

    void precompute_patches(cv::Mat& img, PointCloud& pointcloud, cv::Mat& patch_buf, bool is_derivative);
    double compute_residuals(const Sophus::SE3f& transformation);

    // implementation for LSQNonlinear class
    virtual void update (const ModelType &old_model, ModelType &new_model);


public:
    // weight function
    bool use_weight_scale_;
    float scale_;
    ScaleEstimator::Ptr scale_estimator_;
    WeightFunction::Ptr weight_function_;
    void set_weightfunction();
    void max_level(int level) { max_level_ = level; }

protected:
    virtual double build_LinearSystem(Sophus::SE3f& model);
};
} // namespace dedvo
