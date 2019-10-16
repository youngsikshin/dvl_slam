#pragma once

#include <Eigen/Core>

#include <dedvo/CameraModel.h>
#include <dedvo/PinholeModel.h>
#include <dedvo/Config.h>
#include <dedvo/Datatypes.h>
#include <dedvo/Frame.h>
#include <dedvo/Keyframe.h>


using namespace std;

namespace dedvo
{

class Keyframe;

class LocalTracker {
    enum WeightFunction{TDistribution, Huber};

    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2*patch_halfsize_;
    static const int patch_area_ = patch_size_*patch_size_;

//    static const int pattern_length_ = 8;
//    int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };
    static const int pattern_length_ = 5;
    int pattern_[5][2] = { {0,0}, {1,1}, {-1,-1}, {1, -1}, {-1, 1}};

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LocalTracker(CameraModel::Ptr camera);
    ~LocalTracker();
    bool tracking(vector<Keyframe::Ptr> keyframe_window);

private:
    int min_level_;
    int max_level_;
    int current_level_;

    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

    vector<float> residuals_;

    int iter_;
    int max_iteration_;
    bool stop_;
    size_t n_measurement_;
    float residual_;
    float eps_;

    bool status_;

    Matrix6x6 H_;
    Vector6 Jres_;
    Vector6 x_;

    double compute_residuals_pattern(Keyframe::Ptr keyframe_i, Keyframe::Ptr keyframe_j);

    bool solve();
    void update(Keyframe::Ptr keyframe_j);

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

    bool use_weight_scale_;
    WeightFunction weight_function_;
    float scale_;
    float huber_k_;
    float dof_;

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

} // namespace dedvo
