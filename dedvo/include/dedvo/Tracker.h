#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "CameraModel.h"
#include "PinholeModel.h"
#include "Keyframe.h"
#include "Frame.h"
#include "Config.h"
#include "Datatypes.h"
//#include "util/BotUtils.h"

using namespace std;

namespace dedvo
{

class LeastSquare
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum WeightFunction{TDistribution, Huber};
    
    LeastSquare():
        iter_(0), max_iteration_(100), stop_(false), eps_(0.000001), status_(false), use_weight_scale_(true), scale_(1.0), huber_k_(1.345), dof_(5.0)
    {
        if(use_weight_scale_)
            weight_function_ = TDistribution;
        else
            weight_function_ = Huber;
    }
    ~LeastSquare()
    {

    }

    Matrix6x6& information() { return H_; }

protected:
    int iter_;
    int max_iteration_;
    bool stop_;
    float residual_;
    float eps_;

    bool status_;

    Matrix6x6 prev_H_;
    Matrix6x6 H_;
    Vector6 Jres_;
    Vector6 x_;

    Sophus::SE3f prev_Tji_;

    bool use_weight_scale_;
    WeightFunction weight_function_;
    float scale_;

    virtual double compute_residuals(const Sophus::SE3f& transformation) {};
    virtual double compute_residuals_patterns(const Sophus::SE3f& transformation) {};

public:
    bool solve();
    void update(const Sophus::SE3f& old_Tji, Sophus::SE3f& Tji);
    void optimize(Sophus::SE3f& Tji);

    inline double norm_max(const Vector6& v)
    {
      double max = -1;
      for (int i=0; i<v.size(); i++)
      {
        double abs = fabs(v[i]);
        if(abs>max){
          max = abs;
        }
      }
      return max;
    }

    float huber_weight(const float res);
    float huber_k_;

    float t_dist_weight(const float res);
    float dof_;


    vector<float> errors_;

    float normal_scale(std::vector<float>& errors) const
    {
        const float mean = std::accumulate(errors.begin(), errors.end(), 0)/errors.size();
        float var = 0.0;
        std::for_each(errors.begin(), errors.end(), [&](const float d) {
          var += (d - mean) * (d - mean);
        });
        return std::sqrt(var); // return standard deviation
    }

    float t_dist_scale(std::vector<float>& errors) const
    {
        const float initial_sigma_ = 5.0f;
        float initial_lamda = 1.0f / (initial_sigma_ * initial_sigma_);
        int num = 0;
        float lambda = initial_lamda;
        int iterations = 0;
        do
        {
            ++iterations;
            initial_lamda = lambda;
            num = 0;
            lambda = 0.0f;

            for(std::vector<float>::iterator it=errors.begin(); it!=errors.end(); ++it)
            {
                if(std::isfinite(*it))
                {
                    ++num;
                    const float error2 = (*it)*(*it);
                    lambda += error2 * ( (dof_ + 1.0f) / (dof_ + initial_lamda * error2) );
                }
            }
            lambda = float(num) / lambda;
        } while(std::abs(lambda - initial_lamda) > 1e-3);

        return std::sqrt(1.0f / lambda);

    }
};

class Tracker:public LeastSquare
{
    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2*patch_halfsize_;
    static const int patch_area_ = patch_size_*patch_size_;

    static const int pattern_length_ = 8;
    int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef shared_ptr<Tracker> Ptr;
    
    Tracker();
    ~Tracker();
    bool match(Keyframe::Ptr reference, dedvo::Frame::Ptr current, Sophus::SE3f& transformation);

    vector<bool>& visible_points_in_cur() { return visible_points_in_cur_; }
    vector<bool>& visible_points_in_prev() { return visible_points_in_prev_; }

    float f_num_measurement() { return static_cast<float> (n_measurement_); }

private:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int current_level_;

    int min_level_;
    int max_level_;
    // int max_iteration_;

    // CameraModel camera_;
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    Sophus::SE3f Tji_;

    dedvo::Keyframe::Ptr reference_;
    dedvo::Frame::Ptr current_;

    vector<bool> visible_points_;
    vector<bool> visible_points_in_cur_;
    vector<bool> visible_points_in_prev_;

    bool is_set_ref_patch;
    cv::Mat ref_patch_buf_;
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf_;
//    Eigen::Matrix<float, 6, 1, Eigen::ColMajor> tmp;

    vector<float> weights_;
    vector<size_t> jacobian_buf_idx_;

    // for display
    void show_tracked_result(const Sophus::SE3f& transformation);
    bool display_;

    cv::Mat residual_img_;
    cv::Mat ref_img_with_points;
    cv::Mat cur_img_with_points;

    // optimizer variable
    size_t n_measurement_;

    void precompute_reference_patches();
    void precompute_reference_patterns();

    virtual double compute_residuals(const Sophus::SE3f& transformation);
    virtual double compute_residuals_patterns(const Sophus::SE3f& transformation);
    
    void outlier_rejection(const Sophus::SE3f& transformation);
    void outlier_rejection2(const Sophus::SE3f& transformation);

    void check_visible_points_in_prev (const Sophus::SE3f& transformation);
};

}
