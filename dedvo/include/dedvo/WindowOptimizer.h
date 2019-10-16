#pragma once

#include <Eigen/Core>
#include <Eigen/QR>
#include "CameraModel.h"
#include "PinholeModel.h"
#include "Keyframe.h"
#include "Datatypes.h"

namespace dedvo {

class WindowOptimizer
{
    enum WeightFunction{TDistribution, Huber};

    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2*patch_halfsize_;
    static const int patch_area_ = patch_size_*patch_size_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<WindowOptimizer> Ptr;

    WindowOptimizer(CameraModel::Ptr camera, vector<Keyframe::Ptr> keyframe_window);
    ~WindowOptimizer();
    void optimize();
    void update();
    bool solve();
    void build_LinearSystem();
    double compute_residuals(Keyframe::Ptr keyframe_i, Keyframe::Ptr keyframe_j, Matrix12x12& H, Vector12& Jres);

private:

    vector<Keyframe::Ptr> keyframe_window_;
    int num_keyframe_;

    int iter_;
    int max_iteration_;

    bool use_weight_scale_;
    WeightFunction weight_function_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> scale_sets_;
    float scale_;
    float huber_k_;
    float dof_;

    int pyramid_level_;

    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    // Matrix12x12 H_;
    // Vector12 Jres_;
    // Vector12 x_;
    // Matrix30x30 H_;
    // Vector30 Jres_;
    // Vector30 x_;
    // Matrix60x60 H_;
    // Vector60 Jres_;
    // Vector60 x_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jres_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_;

    bool stop_;

    int n_measurement_;
    double chi2_;
    double residual_;

    vector<float> errors_;
    bool is_add_residuals_;

    float huber_weight(const float res) {
        float sig = 5.0;
        float t_abs = fabsf(res);

        if (t_abs < huber_k_*sig) 
            return 1.0;
        else 
            return (sig*huber_k_) / t_abs;
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

    float t_dist_weight(const float res)
    {
        // return std::max ( (dof_ + 1.0) / (dof_ + res*res), 0.001 );
        return (dof_ + 1.0) / (dof_ + res*res);
    }
};

}   // namespace dedvo
