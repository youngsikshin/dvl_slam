#pragma once

#include <Eigen/Core>
#include <dedvo/LSQNonlinear.hpp>
#include <dedvo/Config.h>
#include <dedvo/KeyframeWindow.h>


namespace dedvo
{

class WindowOptimizer2
{
static const int patch_halfsize_ = 2;
static const int patch_size_ = 2*patch_halfsize_;
static const int patch_area_ = patch_size_*patch_size_;

//static const int pattern_length_ = 8;
//int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };
//static const int pattern_length_ = 5;
//int pattern_[5][2] = { {0,0}, {1,1}, {-1,-1}, {1, -1}, {-1, 1}};
static const int pattern_length_ = 1;
int pattern_[1][2] = { {0,0} };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef shared_ptr<WindowOptimizer2> Ptr;

    WindowOptimizer2(KeyframeWindow::Ptr kf_window);
    ~WindowOptimizer2();

    bool refine();
    bool solve();
    void update();

//    void update (const ModelType &old_model, ModelType &new_model);
    void precompute_patches(cv::Mat& img, PointCloud& pointcloud, cv::Mat& patch_buf, Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>& jacobian_buf, bool is_derivative);

    void compute_residuals(Keyframe::Ptr keyframe_m, Keyframe::Ptr keyframe_n,
                           vector<float>& residuals,
                           vector<Matrix1x6>& frame_jacobian_m, vector<Matrix1x6>& frame_jacobian_n);

protected:
    double build_LinearSystem();//Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& model

private:
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    int iter_;
    int max_iteration_;
    bool stop_;
    double eps_;

    int current_level_;

    int num_keyframe_;
    KeyframeWindow::Ptr kf_window_;

    // weight function
    bool use_weight_scale_;
    float scale_;
    ScaleEstimator::Ptr scale_estimator_;
    WeightFunction::Ptr weight_function_;
    void set_weightfunction();

    int n_measurement_;
    double chi2_;
    double residual_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jres_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_;
};

} //namespace dedvo
