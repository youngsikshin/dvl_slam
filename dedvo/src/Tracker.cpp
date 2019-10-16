#include <dedvo/Tracker.h>
#include <dedvo/Utility.h>
#include <functional>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb.h>
#include <tbb/blocked_range.h>

namespace dedvo
{

bool LeastSquare::solve()
{
    x_ = H_.ldlt().solve(Jres_);

    // if((bool) std::isnan((double) x_[0]))
    //     return false;

    if ( ((x_ - x_).array() == (x_ - x_).array()).all() )
        return true;

    return false;
}

void LeastSquare::update(const Sophus::SE3f& old_Tji, Sophus::SE3f& Tji)
{
    Tji = old_Tji * Sophus::SE3f::exp(-x_);
}

void LeastSquare::optimize(Sophus::SE3f& Tji)
{
    // if(use_weight_) compute_residuals();

    Sophus::SE3f old_Tji = Tji;
    Matrix6x6 old_H = Matrix6x6::Identity();

    stop_ = false;

    cerr << "[LeastSquare]\t start to calc fist residual" << endl;
    compute_residuals_patterns(Tji);
    cerr << "[LeastSquare]\t end to calc fist residual" << endl;

    // double res_diff, old_res_diff, res_diff_diff;

    if(use_weight_scale_)   scale_ = t_dist_scale(errors_);

    for(iter_=0; iter_ < max_iteration_; ++iter_) {
        H_.setZero();
        Jres_.setZero();

        // n_measurement_ = 0;
        double new_residual = compute_residuals_patterns(Tji);

        if(!solve()) {
            cerr << "[LeastSquare] Hessian is closed to singular!" << endl;
            stop_ = true;

            // Tji = old_Tji;
            // break;
        }

        if( (iter_ > 0 && new_residual > residual_) || stop_) {
            Tji = old_Tji;
            H_ = old_H;
//            status_ = false;
            break;
        }

        if(use_weight_scale_) scale_ = t_dist_scale(errors_);

        // if (iter_!=0) {
        //     old_res_diff = res_diff;
        //     res_diff = residual_ - new_residual;
        //     res_diff_diff = res_diff - old_res_diff;           
        // }

        // if ( (iter_!=0 && iter_!=1) && (fabsf(res_diff_diff) < 5e-7)) {
        //     stop_ = true;
        //     Tji = old_Tji;
        //     break;
        // }

        Sophus::SE3f new_Tji;
        update(Tji, new_Tji);
        old_Tji = Tji;
        old_H = H_;
        Tji = new_Tji;

        residual_ = new_residual;

         if (norm_max(x_) < eps_) {
             status_ = true;

             if ( ((x_ - x_).array() != (x_ - x_).array()).all() )
                 status_ = false;

             break;
         }
    }

}

float LeastSquare::huber_weight(const float res)
{
   float t_abs = fabsf(res);
   float sig = 5.0;
    if(t_abs < huber_k_*sig)
        return 1.0;
    else
        return (sig*huber_k_) / t_abs;

}

float LeastSquare::t_dist_weight(const float res)
{
    // return std::max ( (dof_ + 1.0) / (dof_ + res*res), 0.001 );
    return (dof_ + 1.0) / (dof_ + res*res);
}

Tracker::Tracker()
    : LeastSquare(),
      min_level_(Config::cfg()->min_level()),
      max_level_(Config::cfg()->max_level()),
      // max_iteration_(Config::cfg()->max_iterations()),
      n_measurement_(0)
{
    camera_ = Config::cfg()->camera();
    pinhole_model_ = static_pointer_cast<dedvo::PinholeModel> (camera_);
    display_ = true;

    H_.setZero();
}

Tracker::~Tracker()
{

}

bool Tracker::match(Keyframe::Ptr reference, Frame::Ptr current, Sophus::SE3f& transformation)
{
    bool status = false;

    reference_ = reference;
    current_ = current;

    prev_H_ = Matrix6x6::Identity();

//    prev_H_ = H_ / n_measurement_;
//    prev_H_(0,0) = H_(0,0);
//    prev_H_(1,1) = H_(1,1);
//    prev_H_(2,2) = H_(2,2);
//    prev_H_(3,3) = H_(3,3) / n_measurement_;
//    prev_H_(4,4) = H_(4,4) / n_measurement_;
//    prev_H_(5,5) = H_(5,5) / n_measurement_;

    if (H_(0,0) > 1e8)
        cerr << prev_H_.matrix() << endl;

    prev_Tji_ = transformation;
    Tji_ = transformation;//current_->Twc().inverse() * reference_->frame()->Twc();
    
    ref_patch_buf_ = cv::Mat(reference->pointcloud_size(), patch_area_, CV_32F);
    jacobian_buf_.resize(Eigen::NoChange, ref_patch_buf_.rows*patch_area_);

    for(current_level_ = max_level_; current_level_ >= min_level_; current_level_--) {
        visible_points_.clear();
        visible_points_in_cur_.clear();

        visible_points_.resize(ref_patch_buf_.rows, false);
        visible_points_in_cur_.resize(ref_patch_buf_.rows, false);

        jacobian_buf_.setZero();
        is_set_ref_patch = false;

        cerr << "[Tracker]\t Optimizer start!" << endl;
        optimize(Tji_);
        cerr << "[Tracker]\t Optimizer end!" << endl;

        show_tracked_result(Tji_);
    }

    // cerr << H_.matrix() << endl << endl;

    current_level_ = min_level_;

    cerr << "[Tracker]\t" << reference->pointcloud_size() << ", ";
    // outlier_rejection2(Tji_);
    cerr << reference->pointcloud_size() << endl;

    visible_points_in_prev_.clear();
    visible_points_in_prev_.resize(current_->pointcloud().size(),false);
    check_visible_points_in_prev(Tji_);

    // // refinement
    // visible_points_.clear();
    // visible_points_in_cur.clear();

    // visible_points_.resize(ref_patch_buf_.rows, false);
    // visible_points_in_cur.resize(ref_patch_buf_.rows, false);

    // jacobian_buf_.setZero();
    // is_set_ref_patch = false;

    // optimize(Tji_);

    transformation = Tji_;

    return status_;
}

void Tracker::precompute_reference_patches()
{
    const int border = patch_halfsize_+2;
    const cv::Mat& reference_img = reference_->frame()->level(current_level_);
    const int stride = reference_img.cols;
    const float scale = 1.0f/(1<<current_level_);
    // const Vector3d ref_pos = ref_frame_->pos();
    // const double focal_length = ref_frame_->cam_->errorMultiplier2();
    size_t point_counter = 0;
    std::vector<bool>::iterator visibility_iter = visible_points_.begin();

    cv::Mat zbuf;
    zbuf = cv::Mat::zeros(reference_img.size(), CV_8U);

    int num_out_points = 0;

    for(auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++point_counter, ++visibility_iter) {
        auto tic = timestamp_now();

        Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
        Eigen::Vector2f uv_ref;
        uv_ref.noalias() = camera_->xyz_to_uv(xyz_ref) * scale;

        const float u_ref_f = uv_ref(0);
        const float v_ref_f = uv_ref(1);
        const int u_ref_i = static_cast<int> (u_ref_f);
        const int v_ref_i = static_cast<int> (v_ref_f);

        /* If the point does not exist inside the image boundary, it is skipped. */
//        if (u_ref_i - border < 0 || u_ref_i + border > reference_img.cols || v_ref_i - border < 0 || v_ref_i + border > reference_img.rows || xyz_ref(2) <= 0) {
//            num_out_points++;
//            continue;
//        }

        if (zbuf.at<uint8_t>(v_ref_i, u_ref_i) == 1) continue;

        zbuf.at<uint8_t>(v_ref_i, u_ref_i) = 1;

        *visibility_iter = true;

        // // evaluate projection jacobian
        Matrix2x6 frame_jac;
        Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

        const float subpix_u_ref = u_ref_f-u_ref_i;
        const float subpix_v_ref = v_ref_f-v_ref_i;
        const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
        const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
        const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
        const float w_ref_br = subpix_u_ref * subpix_v_ref;

        size_t pixel_counter = 0;
        float* ref_patch_buf_ptr = reinterpret_cast<float *> (ref_patch_buf_.data) + patch_area_ * point_counter;

        auto toc = timestamp_now();

        auto tic2 = timestamp_now();
        for (int y=0; y<patch_size_; ++y) {
            float *reference_img_ptr = (float*) reference_img.data + (v_ref_i+y - patch_halfsize_)*stride  + (u_ref_i - patch_halfsize_);

            // interpolated reference patch color
            for (int x=0; x<patch_size_; ++x, ++reference_img_ptr, ++ref_patch_buf_ptr, ++pixel_counter) {
                *ref_patch_buf_ptr = w_ref_tl * reference_img_ptr[0] + w_ref_tr * reference_img_ptr[1]
                                    + w_ref_bl * reference_img_ptr[stride] + w_ref_br * reference_img_ptr[stride+1];
                // *ref_patch_buf_ptr = reference_img_ptr[0];

                // precompute image gradient
                float dx = 0.5f * ((w_ref_tl*reference_img_ptr[1] + w_ref_tr*reference_img_ptr[2] + w_ref_bl*reference_img_ptr[stride+1] + w_ref_br*reference_img_ptr[stride+2])
                          -(w_ref_tl*reference_img_ptr[-1] + w_ref_tr*reference_img_ptr[0] + w_ref_bl*reference_img_ptr[stride-1] + w_ref_br*reference_img_ptr[stride]));
                float dy = 0.5f * ((w_ref_tl*reference_img_ptr[stride] + w_ref_tr*reference_img_ptr[1+stride] + w_ref_bl*reference_img_ptr[stride*2] + w_ref_br*reference_img_ptr[stride*2+1])
                          -(w_ref_tl*reference_img_ptr[-stride] + w_ref_tr*reference_img_ptr[1-stride] + w_ref_bl*reference_img_ptr[0] + w_ref_br*reference_img_ptr[1]));

                // float dx = 0.5f * (reference_img_ptr[1]-reference_img_ptr[-1]);
                // float dy = 0.5f * (reference_img_ptr[stride]-reference_img_ptr[-stride]);

                // push to jacobian buffer
                jacobian_buf_.col(point_counter*patch_area_ + pixel_counter) = (dx*pinhole_model_->fx()*frame_jac.row(0) + dy*pinhole_model_->fy()*frame_jac.row(1)) / (1<<current_level_);
            }
        }
        auto toc2 = timestamp_now();

//        cerr << toc - tic << endl;
//        cerr << toc2 - tic2 << endl;


    }

    is_set_ref_patch = true;
}

void Tracker::check_visible_points_in_prev (const Sophus::SE3f& transformation)
{
    const int border = patch_halfsize_+2;
    cv::Mat& current_img = current_->level(current_level_);

    const float scale = 1.0f/(1<<current_level_);
    std::vector<bool>::iterator visibility_iter_prev = visible_points_in_prev_.begin();

    for (auto iter=current_->pointcloud().begin(); iter!=current_->pointcloud().end(); ++iter, ++visibility_iter_prev) {
        Eigen::Vector3f xyz_cur (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_prev = transformation.inverse()*xyz_cur;
        Eigen::Vector2f uv_prev;
        uv_prev.noalias() = camera_->xyz_to_uv(xyz_prev) * scale;

        const float u_prev_f = uv_prev(0);
        const float v_prev_f = uv_prev(1);
        const int u_prev_i = static_cast<int> (u_prev_f);
        const int v_prev_i = static_cast<int> (v_prev_f);

        if (u_prev_i - border < 0 || u_prev_i + border > current_img.cols || v_prev_i - border < 0 || v_prev_i + border > current_img.rows || xyz_prev(2) <= 0)
            continue;

        *visibility_iter_prev = true;
    }
}

double Tracker::compute_residuals(const Sophus::SE3f& transformation)
{
    n_measurement_ = 0; 
    errors_.clear();

    cv::Mat& current_img = current_->level(current_level_);

    if(display_) 
        residual_img_ = cv::Mat(current_img.size(), CV_32F, cv::Scalar(0));

    if(!is_set_ref_patch) {
        // uint64_t tic = timestamp_now();
        precompute_reference_patches();
        // copy(visible_points_.begin(), visible_points_.end(), visible_points_in_cur.begin());
        // uint64_t toc = timestamp_now();
        // cout << (toc-tic)/1e6 << endl;
    }

    // compute the weights on the first iteration
    // std::vector<float> errors;
    // if(compute_weight_scale)

    errors_.reserve(visible_points_.size());

    
    const int stride = current_img.cols;
    const int border = patch_halfsize_+2;
    const float scale = 1.0f/(1<<current_level_);

    float chi2 = 0.0;
    size_t point_counter = 0; // is used to compute the index of the cached jacobian
    std::vector<bool>::iterator visibility_iter = visible_points_.begin();
    std::vector<bool>::iterator visibility_iter_cur = visible_points_in_cur_.begin();

    for (auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++point_counter, ++visibility_iter, ++visibility_iter_cur) {
        if(!*visibility_iter)
            continue;

        Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_cur = transformation*xyz_ref;
        Eigen::Vector2f uv_cur;
        uv_cur.noalias() = camera_->xyz_to_uv(xyz_cur) * scale;
        
        const float u_cur_f = uv_cur(0);
        const float v_cur_f = uv_cur(1);
        const int u_cur_i = static_cast<int> (u_cur_f);
        const int v_cur_i = static_cast<int> (v_cur_f);

        if (u_cur_i - border < 0 || u_cur_i + border > current_img.cols || v_cur_i - border < 0 || v_cur_i + border > current_img.rows || xyz_cur(2) <= 0)
            continue;

        *visibility_iter_cur = true;

        const float subpix_u_cur = u_cur_f-u_cur_i;
        const float subpix_v_cur = v_cur_f-v_cur_i;
        const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
        const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
        const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
        const float w_cur_br = subpix_u_cur * subpix_v_cur;

        float* ref_patch_buf_ptr = reinterpret_cast<float*>(ref_patch_buf_.data) + patch_area_*point_counter;

        size_t pixel_counter = 0;
    
        // Eigen::Matrix<float, 16, 16> weights;
        // Eigen::Matrix<float, 16, 1> residuals;
        // weights.setZero();
        // residuals.setZero();

        for (int y=0; y<patch_size_; ++y) {
            float* current_img_ptr = (float*) current_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

            for (int x=0; x<patch_size_; ++x, ++current_img_ptr, ++ref_patch_buf_ptr, ++pixel_counter) {
                float intensity_cur = w_cur_tl*current_img_ptr[0] + w_cur_tr*current_img_ptr[1] + w_cur_bl*current_img_ptr[stride] + w_cur_br*current_img_ptr[stride+1];
                float res = (intensity_cur - (*ref_patch_buf_ptr));

                // robustification
                float weight = 1.0;

                
                if(use_weight_scale_)
                    weight = t_dist_weight(res/scale_);
                else
                    weight = huber_weight(res/scale_);

                errors_.push_back(res);


                // weights(pixel_counter, pixel_counter) = weight;
                // residuals(pixel_counter) = res;

                chi2 += res*res*weight;

                if (current_level_ == 4) {
                    // cerr << intensity_cur << " - " << (*ref_patch_buf_ptr) << endl; 
                    // if ((n_measurement_ % 10000 == 0)) {
                    // if (chi2 > 1e10) {
                    //     cerr << intensity_cur << " - " << (*ref_patch_buf_ptr) << endl; 
                    //     cerr << chi2 << ", " << res << ", " << weight << endl;
                    // }
                    // if ((*ref_patch_buf_ptr) > 10000) cerr << *ref_patch_buf_ptr << ", " << chi2 << ", " << res << ", " << weight << endl;
                }
                
                n_measurement_++;

                // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
                Vector6 J(jacobian_buf_.col(point_counter*patch_area_ + pixel_counter));
                H_.noalias() += J*J.transpose()*weight;
                Jres_.noalias() -= J*res*weight;

                // residuals_.push_back(res);
                // weights_.push_back(weight);
                // jacobian_buf_idx_.push_back(point_counter*patch_area_ + pixel_counter);

                if(display_)
                    residual_img_.at<float>((int) v_cur_f+y-patch_halfsize_, (int) u_cur_f+x-patch_halfsize_) = res;///255.0;

            }

        }

        // Eigen::Matrix<float, 6, 16> Jblock(jacobian_buf_.block<6, 16>(0, point_counter*patch_area_));
        // H_.noalias() += Jblock*weights*Jblock.transpose();
        // Jres_.noalias() -= Jblock*weights*residuals;
    }

    // if(isinf(chi2)) cerr << chi2 << " / " << n_measurement_ << " = " << chi2/n_measurement_ << endl;

    H_ = H_;
    Jres_ = Jres_;

    return chi2/n_measurement_;
}

void Tracker::precompute_reference_patterns()
{
    const int border = patch_halfsize_+2;
    const cv::Mat& reference_img = reference_->frame()->level(current_level_);
    const int stride = reference_img.cols;
    const float scale = 1.0f/(1<<current_level_);
    // const Vector3d ref_pos = ref_frame_->pos();
    // const double focal_length = ref_frame_->cam_->errorMultiplier2();
    size_t point_counter = 0;
//    std::vector<bool>::iterator visibility_iter = visible_points_.begin();

    cv::Mat zbuf;
    zbuf = cv::Mat::zeros(reference_img.size(), CV_8U);

    int num_out_points = 0;

//    for(auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++point_counter, ++visibility_iter) {
    for(auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++point_counter) {
        auto tic = timestamp_now();

        Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
        Eigen::Vector2f uv_ref;
        uv_ref.noalias() = camera_->xyz_to_uv(xyz_ref) * scale;

        const float u_ref_f = uv_ref(0);
        const float v_ref_f = uv_ref(1);
        const int u_ref_i = static_cast<int> (u_ref_f);
        const int v_ref_i = static_cast<int> (v_ref_f);

        /* If the point does not exist inside the image boundary, it is skipped. */
//        if (u_ref_i - border < 0 || u_ref_i + border > reference_img.cols || v_ref_i - border < 0 || v_ref_i + border > reference_img.rows || xyz_ref(2) <= 0) {
//            num_out_points++;
//            continue;
//        }

//        if (zbuf.at<uint8_t>(v_ref_i, u_ref_i) == 1) continue;

//        zbuf.at<uint8_t>(v_ref_i, u_ref_i) = 1;

//        *visibility_iter = true;

        // // evaluate projection jacobian
        Matrix2x6 frame_jac;
        Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

        const float subpix_u_ref = u_ref_f-u_ref_i;
        const float subpix_v_ref = v_ref_f-v_ref_i;
        const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
        const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
        const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
        const float w_ref_br = subpix_u_ref * subpix_v_ref;

        size_t pixel_counter = 0;
//        float* ref_patch_buf_ptr = reinterpret_cast<float *> (ref_patch_buf_.data) + patch_area_ * point_counter;

//        for (int y=0; y<patch_size_; ++y) {
//            float *reference_img_ptr = (float*) reference_img.data + (v_ref_i+y - patch_halfsize_)*stride  + (u_ref_i - patch_halfsize_);

//            // interpolated reference patch color
//            for (int x=0; x<patch_size_; ++x, ++reference_img_ptr, ++ref_patch_buf_ptr, ++pixel_counter) {
//                *ref_patch_buf_ptr = w_ref_tl * reference_img_ptr[0] + w_ref_tr * reference_img_ptr[1]
//                                    + w_ref_bl * reference_img_ptr[stride] + w_ref_br * reference_img_ptr[stride+1];
//                // *ref_patch_buf_ptr = reference_img_ptr[0];

//                // precompute image gradient
//                float dx = 0.5f * ((w_ref_tl*reference_img_ptr[1] + w_ref_tr*reference_img_ptr[2] + w_ref_bl*reference_img_ptr[stride+1] + w_ref_br*reference_img_ptr[stride+2])
//                          -(w_ref_tl*reference_img_ptr[-1] + w_ref_tr*reference_img_ptr[0] + w_ref_bl*reference_img_ptr[stride-1] + w_ref_br*reference_img_ptr[stride]));
//                float dy = 0.5f * ((w_ref_tl*reference_img_ptr[stride] + w_ref_tr*reference_img_ptr[1+stride] + w_ref_bl*reference_img_ptr[stride*2] + w_ref_br*reference_img_ptr[stride*2+1])
//                          -(w_ref_tl*reference_img_ptr[-stride] + w_ref_tr*reference_img_ptr[1-stride] + w_ref_bl*reference_img_ptr[0] + w_ref_br*reference_img_ptr[1]));

//                // float dx = 0.5f * (reference_img_ptr[1]-reference_img_ptr[-1]);
//                // float dy = 0.5f * (reference_img_ptr[stride]-reference_img_ptr[-stride]);

//                // push to jacobian buffer
//                jacobian_buf_.col(point_counter*patch_area_ + pixel_counter) = (dx*pinhole_model_->fx()*frame_jac.row(0) + dy*pinhole_model_->fy()*frame_jac.row(1)) / (1<<current_level_);
//            }
//        }


        float* ref_patch_buf_ptr = reinterpret_cast<float *> (ref_patch_buf_.data) + pattern_length_ * point_counter;

        for (int i=0; i < pattern_length_; ++i, ++pixel_counter, ++ref_patch_buf_ptr) {
            int x = pattern_[i][0];
            int y = pattern_[i][1];

            float *reference_img_ptr = (float*) reference_img.data + (v_ref_i+y)*stride  + (u_ref_i+x);
            *ref_patch_buf_ptr = w_ref_tl * reference_img_ptr[0] + w_ref_tr * reference_img_ptr[1]
                                + w_ref_bl * reference_img_ptr[stride] + w_ref_br * reference_img_ptr[stride+1];

            // precompute image gradient
            float dx = 0.5f * ((w_ref_tl*reference_img_ptr[1] + w_ref_tr*reference_img_ptr[2] + w_ref_bl*reference_img_ptr[stride+1] + w_ref_br*reference_img_ptr[stride+2])
                      -(w_ref_tl*reference_img_ptr[-1] + w_ref_tr*reference_img_ptr[0] + w_ref_bl*reference_img_ptr[stride-1] + w_ref_br*reference_img_ptr[stride]));
            float dy = 0.5f * ((w_ref_tl*reference_img_ptr[stride] + w_ref_tr*reference_img_ptr[1+stride] + w_ref_bl*reference_img_ptr[stride*2] + w_ref_br*reference_img_ptr[stride*2+1])
                      -(w_ref_tl*reference_img_ptr[-stride] + w_ref_tr*reference_img_ptr[1-stride] + w_ref_bl*reference_img_ptr[0] + w_ref_br*reference_img_ptr[1]));


            jacobian_buf_.col(point_counter*pattern_length_ + pixel_counter) = (dx*pinhole_model_->fx()*frame_jac.row(0) + dy*pinhole_model_->fy()*frame_jac.row(1)) / (1<<current_level_);
//            cerr << pattern_[i][0] << ", " << pattern_[i][1] << endl;
        }

//        cerr << toc - tic << endl;
//        cerr << toc2 - tic2 << endl;


    }

    is_set_ref_patch = true;
}

double Tracker::compute_residuals_patterns(const Sophus::SE3f& transformation)
{
    n_measurement_ = 0;
    errors_.clear();

    cv::Mat& current_img = current_->level(current_level_);

    if(display_)
        residual_img_ = cv::Mat(current_img.size(), CV_32F, cv::Scalar(0));

    if(!is_set_ref_patch) {
        // uint64_t tic = timestamp_now();
        precompute_reference_patterns();
        // copy(visible_points_.begin(), visible_points_.end(), visible_points_in_cur.begin());
        // uint64_t toc = timestamp_now();
        // cout << (toc-tic)/1e6 << endl;
    }

    // compute the weights on the first iteration
    // std::vector<float> errors;
    // if(compute_weight_scale)

    errors_.reserve(visible_points_.size());


    const int stride = current_img.cols;
    const int border = patch_halfsize_+2;
    const float scale = 1.0f/(1<<current_level_);

    float chi2 = 0.0;
    size_t point_counter = 0; // is used to compute the index of the cached jacobian
    std::vector<bool>::iterator visibility_iter = visible_points_.begin();
    std::vector<bool>::iterator visibility_iter_cur = visible_points_in_cur_.begin();

//    for (auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++point_counter, ++visibility_iter, ++visibility_iter_cur) {
    for (auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++point_counter, ++visibility_iter_cur) {
//        if(!*visibility_iter)
//            continue;

        Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_cur = transformation*xyz_ref;
        Eigen::Vector2f uv_cur;
        uv_cur.noalias() = camera_->xyz_to_uv(xyz_cur) * scale;

        const float u_cur_f = uv_cur(0);
        const float v_cur_f = uv_cur(1);
        const int u_cur_i = static_cast<int> (u_cur_f);
        const int v_cur_i = static_cast<int> (v_cur_f);

        if (u_cur_i - border < 0 || u_cur_i + border > current_img.cols || v_cur_i - border < 0 || v_cur_i + border > current_img.rows || xyz_cur(2) <= 0)
            continue;

        *visibility_iter_cur = true;

        const float subpix_u_cur = u_cur_f-u_cur_i;
        const float subpix_v_cur = v_cur_f-v_cur_i;
        const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
        const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
        const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
        const float w_cur_br = subpix_u_cur * subpix_v_cur;

        float* ref_patch_buf_ptr = reinterpret_cast<float*>(ref_patch_buf_.data) + pattern_length_*point_counter;

        size_t pixel_counter = 0;

        // Eigen::Matrix<float, 16, 16> weights;
        // Eigen::Matrix<float, 16, 1> residuals;
        // weights.setZero();
        // residuals.setZero();

//        for (int y=0; y<patch_size_; ++y) {
//            float* current_img_ptr = (float*) current_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

//            for (int x=0; x<patch_size_; ++x, ++current_img_ptr, ++ref_patch_buf_ptr, ++pixel_counter) {
//                float intensity_cur = w_cur_tl*current_img_ptr[0] + w_cur_tr*current_img_ptr[1] + w_cur_bl*current_img_ptr[stride] + w_cur_br*current_img_ptr[stride+1];
//                float res = (intensity_cur - (*ref_patch_buf_ptr));

//                // robustification
//                float weight = 1.0;


//                if(use_weight_scale_)
//                    weight = t_dist_weight(res/scale_);
//                else
//                    weight = huber_weight(res/scale_);

//                errors_.push_back(res);


//                // weights(pixel_counter, pixel_counter) = weight;
//                // residuals(pixel_counter) = res;

//                chi2 += res*res*weight;

//                n_measurement_++;

//                // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
//                Vector6 J(jacobian_buf_.col(point_counter*pattern_length_ + pixel_counter));
//                H_.noalias() += J*J.transpose()*weight;
//                Jres_.noalias() -= J*res*weight;

//                // residuals_.push_back(res);
//                // weights_.push_back(weight);
//                // jacobian_buf_idx_.push_back(point_counter*patch_area_ + pixel_counter);

//                if(display_)
//                    residual_img_.at<float>((int) v_cur_f+y-patch_halfsize_, (int) u_cur_f+x-patch_halfsize_) = res;///255.0;

//            }

//        }

        for (int i=0; i < pattern_length_; ++i, ++pixel_counter, ++ref_patch_buf_ptr) {
            int x = pattern_[i][0];
            int y = pattern_[i][1];

            float* current_img_ptr = (float*) current_img.data + (v_cur_i+y)*stride + (u_cur_i+x);

            float intensity_cur = w_cur_tl*current_img_ptr[0] + w_cur_tr*current_img_ptr[1] + w_cur_bl*current_img_ptr[stride] + w_cur_br*current_img_ptr[stride+1];
            float res = (intensity_cur - (*ref_patch_buf_ptr));

            // robustification
            float weight = 1.0;


            if(use_weight_scale_)
                weight = t_dist_weight(res/scale_);
            else
                weight = huber_weight(res/scale_);

            errors_.push_back(res);


            // weights(pixel_counter, pixel_counter) = weight;
            // residuals(pixel_counter) = res;

            chi2 += res*res*weight;

            n_measurement_++;

            // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
            Vector6 J(jacobian_buf_.col(point_counter*pattern_length_ + pixel_counter));
            H_.noalias() += J*J.transpose()*weight;// + prev_H_ * weight;
            Jres_.noalias() -= J*res*weight;// + prev_H_ * (prev_Tji_.inverse() * transformation).log() * weight;

            // residuals_.push_back(res);
            // weights_.push_back(weight);
            // jacobian_buf_idx_.push_back(point_counter*patch_area_ + pixel_counter);

            if(display_)
                residual_img_.at<float>((int) v_cur_f+y-patch_halfsize_, (int) u_cur_f+x-patch_halfsize_) = res;///255.0;

        }

        // Eigen::Matrix<float, 6, 16> Jblock(jacobian_buf_.block<6, 16>(0, point_counter*patch_area_));
        // H_.noalias() += Jblock*weights*Jblock.transpose();
        // Jres_.noalias() -= Jblock*weights*residuals;
    }

    // if(isinf(chi2)) cerr << chi2 << " / " << n_measurement_ << " = " << chi2/n_measurement_ << endl;

    H_ = H_;
    Jres_ = Jres_;

    return chi2/n_measurement_;
}

void Tracker::outlier_rejection(const Sophus::SE3f& transformation)
{
    cv::Mat& current_img = current_->level(current_level_);

    if(display_) 
        residual_img_ = cv::Mat(current_img.size(), CV_32F, cv::Scalar(0));

    if(!is_set_ref_patch) {
        precompute_reference_patches();
    }

    const int stride = current_img.cols;
    const int border = patch_halfsize_+2;
    const float scale = 1.0f/(1<<current_level_);

    size_t point_counter = 0; // is used to compute the index of the cached jacobian
    std::vector<bool>::iterator visibility_iter = visible_points_.begin();
    std::vector<bool>::iterator visibility_iter_cur = visible_points_in_cur_.begin();

    for (auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++point_counter, ++visibility_iter, ++visibility_iter_cur) {
        if(!*visibility_iter) {
            ++iter;
            continue;
        }

        Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_cur = transformation*xyz_ref;
        Eigen::Vector2f uv_cur;
        uv_cur.noalias() = camera_->xyz_to_uv(xyz_cur) * scale;
        
        const float u_cur_f = uv_cur(0);
        const float v_cur_f = uv_cur(1);
        const int u_cur_i = static_cast<int> (u_cur_f);
        const int v_cur_i = static_cast<int> (v_cur_f);

        if (u_cur_i - border < 0 || u_cur_i + border > current_img.cols || v_cur_i - border < 0 || v_cur_i + border > current_img.rows || xyz_cur(2) <= 0) {
            ++iter;
            continue;
        }

        *visibility_iter_cur = true;

        const float subpix_u_cur = u_cur_f-u_cur_i;
        const float subpix_v_cur = v_cur_f-v_cur_i;
        const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
        const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
        const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
        const float w_cur_br = subpix_u_cur * subpix_v_cur;

        float* ref_patch_buf_ptr = reinterpret_cast<float*>(ref_patch_buf_.data) + patch_area_*point_counter;

        size_t pixel_counter = 0;
    
        float res = 0;

        for (int y=0; y<patch_size_; ++y) {
            float* current_img_ptr = (float*) current_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

            for (int x=0; x<patch_size_; ++x, ++current_img_ptr, ++ref_patch_buf_ptr, ++pixel_counter) {
                float intensity_cur = w_cur_tl*current_img_ptr[0] + w_cur_tr*current_img_ptr[1] + w_cur_bl*current_img_ptr[stride] + w_cur_br*current_img_ptr[stride+1];
                res += 255*(intensity_cur - (*ref_patch_buf_ptr));
            }

        }

        res = res/static_cast<float>(patch_size_*patch_size_);

        float t_abs = fabsf(res);
        float sig = 1.0;
        
        if(t_abs < huber_k_*sig)
            ++iter;
        else
            iter = reference_->pointcloud().erase(iter);

    }

}

void Tracker::outlier_rejection2(const Sophus::SE3f& transformation)
{
    cv::Mat& current_img = current_->level(current_level_);

    if(display_) 
        residual_img_ = cv::Mat(current_img.size(), CV_32F, cv::Scalar(0));

    if(!is_set_ref_patch) {
        precompute_reference_patches();
    }
    
    const int stride = current_img.cols;
    const int border = patch_halfsize_+2;
    const float scale = 1.0f/(1<<current_level_);

    size_t point_counter = 0; // is used to compute the index of the cached jacobian
    std::vector<bool>::iterator visibility_iter = visible_points_.begin();
    std::vector<bool>::iterator visibility_iter_cur = visible_points_in_cur_.begin();

    int point_idx=0;
    vector<pair<Matrix6x6, int>> H_pair;
    vector<pair<float, int>> logdetH_pair;

    for (auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++point_counter, ++visibility_iter, ++visibility_iter_cur, ++point_idx) {
        if(!*visibility_iter) {
            ++iter;
            continue;
        }

        Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_cur = transformation*xyz_ref;
        Eigen::Vector2f uv_cur;
        uv_cur.noalias() = camera_->xyz_to_uv(xyz_cur) * scale;
        
        const float u_cur_f = uv_cur(0);
        const float v_cur_f = uv_cur(1);
        const int u_cur_i = static_cast<int> (u_cur_f);
        const int v_cur_i = static_cast<int> (v_cur_f);

        if (u_cur_i - border < 0 || u_cur_i + border > current_img.cols || v_cur_i - border < 0 || v_cur_i + border > current_img.rows || xyz_cur(2) <= 0) {
            ++iter;
            continue;
        }

        *visibility_iter_cur = true;

        const float subpix_u_cur = u_cur_f-u_cur_i;
        const float subpix_v_cur = v_cur_f-v_cur_i;
        const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
        const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
        const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
        const float w_cur_br = subpix_u_cur * subpix_v_cur;

        float* ref_patch_buf_ptr = reinterpret_cast<float*>(ref_patch_buf_.data) + patch_area_*point_counter;

        size_t pixel_counter = 0;
    
        float res = 0;
        Matrix6x6 H;
        H.setZero();

        for (int y=0; y<patch_size_; ++y) {
            float* current_img_ptr = (float*) current_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

            for (int x=0; x<patch_size_; ++x, ++current_img_ptr, ++ref_patch_buf_ptr, ++pixel_counter) {
                float intensity_cur = w_cur_tl*current_img_ptr[0] + w_cur_tr*current_img_ptr[1] + w_cur_bl*current_img_ptr[stride] + w_cur_br*current_img_ptr[stride+1];
                res += 255*(intensity_cur - (*ref_patch_buf_ptr));

                float weight = 1.0;
                if(use_weight_scale_)
                    weight = t_dist_weight(res/scale_);
                else
                    weight = huber_weight(res/scale_);

                Vector6 J(jacobian_buf_.col(point_counter*patch_area_ + pixel_counter));
                H.noalias() += J*J.transpose()*weight;

            }
        }

        res = res/static_cast<float>(patch_size_*patch_size_);
        H = H/static_cast<float>(patch_size_*patch_size_);
        H_pair.push_back(pair<Matrix6x6,int>(H,point_idx));

        float logdetH = 0.0;
        for(int i=0; i<6; i++) {
            logdetH += log(H(i,i));
        }

        logdetH_pair.push_back(pair<float, int>(logdetH,point_idx));
        
        ++iter;
    }

    sort(logdetH_pair.begin(),logdetH_pair.end(), greater<pair<float,int>>());

    point_idx = 0;
    for (auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++point_idx) {

        bool exist = false;
        for(int i=0; i< 10; i++) {
            if ( point_idx == logdetH_pair[i].second ) {
                exist = true;
                break;
            }
        }

        if(exist) ++iter;
        else    iter = reference_->pointcloud().erase(iter);
    }

    // cerr << "Point size: " << reference_->pointcloud().size() << endl;
}


void Tracker::show_tracked_result(const Sophus::SE3f& transformation)
{
    cerr << "[Tracker]\t" << current_level_ << " " << iter_ << " "  << residual_ << " " << scale_ << endl;

    if (current_level_== min_level_) {

        ref_img_with_points = cv::Mat(cv::Size(reference_->frame()->level(current_level_).cols, reference_->frame()->level(current_level_).rows), CV_8UC3);
        cvtColor(reference_->frame()->level(current_level_), ref_img_with_points, cv::COLOR_GRAY2BGR);

        cur_img_with_points = cv::Mat(cv::Size(current_->level(current_level_).cols, current_->level(current_level_).rows), CV_8UC3);
        cvtColor(current_->level(current_level_), cur_img_with_points, cv::COLOR_GRAY2BGR);

        const float scale = 1.0f/(1<<current_level_);
        std::vector<bool>::iterator visibility_iter = visible_points_in_cur_.begin();
        for(auto iter=reference_->pointcloud().begin(); iter!=reference_->pointcloud().end(); ++iter, ++visibility_iter) {
            if(*visibility_iter){
                Eigen::Vector3f xyz_ref (iter->x, iter->y, iter->z);
                Eigen::Vector2f uv_ref;
                uv_ref.noalias() = camera_->xyz_to_uv(xyz_ref) * scale;

                const float u_ref_f = uv_ref(0);
                const float v_ref_f = uv_ref(1);
                const int u_ref_i = static_cast<int> (u_ref_f);
                const int v_ref_i = static_cast<int> (v_ref_f);

                Eigen::Vector3f xyz_cur = transformation*xyz_ref;
                Eigen::Vector2f uv_cur;
                uv_cur.noalias() = camera_->xyz_to_uv(xyz_cur) * scale;
            
                const float u_cur_f = uv_cur(0);
                const float v_cur_f = uv_cur(1);
                const int u_cur_i = static_cast<int> (u_cur_f);
                const int v_cur_i = static_cast<int> (v_cur_f);

                cv::circle(ref_img_with_points, cv::Point(u_ref_i, v_ref_i), 0, cv::Scalar(0,0,255), -1);
                cv::circle(cur_img_with_points, cv::Point(u_cur_i, v_cur_i), 0, cv::Scalar(0,0,255), -1);
            }
        }

        cv::namedWindow("cur_img_with_points",cv::WINDOW_NORMAL);
        cv::imshow("cur_img_with_points",cur_img_with_points);

        cv::namedWindow("residual_img_",cv::WINDOW_NORMAL);
        cv::imshow("residual_img_",residual_img_);

        cv::waitKey(1);

    }
}

}
