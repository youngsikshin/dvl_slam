#include <dedvo/Tracker2.h>

namespace dedvo
{

Tracker2::Tracker2()
{
    camera_ = Config::cfg()->camera();
    pinhole_model_ = static_pointer_cast<dedvo::PinholeModel> (camera_);

    auto tracker_info = Config::cfg()->tracker();
    min_level_ = tracker_info.min_level;
    max_level_ = tracker_info.max_level;

    use_weight_scale_ = tracker_info.use_weight_scale;

    set_weightfunction();
}

Tracker2::~Tracker2()
{

}

void Tracker2::set_weightfunction()
{
    auto tracker_info = Config::cfg()->tracker();
    if(use_weight_scale_) {
        switch(tracker_info.scale_estimator_type)
        {
            case  ScaleEstimatorType::TDistributionScale:
                scale_estimator_.reset(new TDistributionScaleEstimator());
                break;
            default:
                cerr << "Do not use scale estimator." << endl;
        }
    }

    switch(tracker_info.weight_function_type)
    {
        case WeightFunctionType::TDistributionWeight:
            weight_function_.reset(new TDistributionWeightFunction());
            break;
        default:
            cerr << "Do not use weight function." << endl;
    }

}

bool Tracker2::tracking(Keyframe::Ptr reference, Frame::Ptr current, Sophus::SE3f& transformation)
{
    bool status = true;

    reference_ = reference;
    current_ = current;

    affine_a_ = 1.0f;
    affine_b_ = 0.0f;

    for(current_level_ = max_level_; current_level_ >= min_level_; current_level_--) {
        is_precomputed_ = false;
        stop_ = false;
//        mu_ = 0.1f;

        optimize(transformation);
    }

    return status;
}

void Tracker2::update (const ModelType& old_model, ModelType& new_model)
{
    new_model = old_model * Sophus::SE3f::exp(-x_);

//    cerr << "[Tracker2]\t The model was updated." << endl;
//    cerr << new_model.matrix() << endl << endl;
}

void Tracker2::precompute_patches(cv::Mat& img, PointCloud& pointcloud, cv::Mat& patch_buf, bool is_derivative)
{
    const int border = patch_halfsize_+2+2;
    const int stride = img.cols;
    const float scale = 1.0f/(1<<current_level_);

    cv::Mat zbuf;
    zbuf = cv::Mat::zeros(img.size(), CV_8U);

    vector<Eigen::Vector2f> uv_set = camera_->pointcloud_to_uv(pointcloud, scale);

    patch_buf = cv::Mat(pointcloud.size(), pattern_length_, CV_32F);

    if(is_derivative) {
        dI_buf_.resize(Eigen::NoChange, patch_buf.rows*pattern_length_);
        jacobian_buf_.resize(Eigen::NoChange, patch_buf.rows*pattern_length_);

        jacobian_buf_.setZero();
    }

    auto pc_iter = pointcloud.begin();
    size_t point_counter = 0;

    for(auto uv_iter=uv_set.begin(); uv_iter != uv_set.end(); ++uv_iter, ++pc_iter, ++point_counter) {
        Eigen::Vector2f& uv = *uv_iter;
        float& u_f = uv(0);
        float& v_f = uv(1);
        const int u_i = static_cast<int> (u_f);
        const int v_i = static_cast<int> (v_f);

        if (u_i - border < 0 || u_i + border > img.cols || v_i - border < 0 || v_i + border > img.rows || pc_iter->z <= 0.0) {
            float* patch_buf_ptr = reinterpret_cast<float *> (patch_buf.data) + pattern_length_ * point_counter;
            for(int i=0; i<pattern_length_; ++i, ++patch_buf_ptr)
                *patch_buf_ptr = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        const float subpix_u = u_f-u_i;
        const float subpix_v = v_f-v_i;
        const float w_tl = (1.0-subpix_u) * (1.0-subpix_v);
        const float w_tr = subpix_u * (1.0-subpix_v);
        const float w_bl = (1.0-subpix_u) * subpix_v;
        const float w_br = subpix_u * subpix_v;


        size_t pixel_counter = 0;

        float* patch_buf_ptr = reinterpret_cast<float *> (patch_buf.data) + pattern_length_ * point_counter;

        for (int i=0; i < pattern_length_; ++i, ++pixel_counter, ++patch_buf_ptr) {
            int x = pattern_[i][0];
            int y = pattern_[i][1];

            float* img_ptr = (float*) img.data + (v_i+y)*stride  + (u_i+x);
            *patch_buf_ptr = w_tl * img_ptr[0] + w_tr * img_ptr[1]
                                + w_bl * img_ptr[stride] + w_br * img_ptr[stride+1];

            if(is_derivative) {
                // precompute image gradient
                float dx = 0.5f * ((w_tl*img_ptr[1] + w_tr*img_ptr[2] + w_bl*img_ptr[stride+1] + w_br*img_ptr[stride+2])
                          -(w_tl*img_ptr[-1] + w_tr*img_ptr[0] + w_bl*img_ptr[stride-1] + w_br*img_ptr[stride]));
                float dy = 0.5f * ((w_tl*img_ptr[stride] + w_tr*img_ptr[1+stride] + w_bl*img_ptr[stride*2] + w_br*img_ptr[stride*2+1])
                          -(w_tl*img_ptr[-stride] + w_tr*img_ptr[1-stride] + w_bl*img_ptr[0] + w_br*img_ptr[1]));

                Matrix2x6 frame_jac;
                Eigen::Vector3f xyz(pc_iter->x, pc_iter->y, pc_iter->z);
                Frame::jacobian_xyz2uv(xyz, frame_jac);

                Eigen::Vector2f dI_xy(dx, dy);
                dI_buf_.col(point_counter*pattern_length_ + i) = dI_xy;
                jacobian_buf_.col(point_counter*pattern_length_ + pixel_counter) = (dx*pinhole_model_->fx()*frame_jac.row(0) + dy*pinhole_model_->fy()*frame_jac.row(1)) / (1<<current_level_);
            }

        }
    }
}


double Tracker2::compute_residuals(const Sophus::SE3f& transformation)
{
    errors_.clear();
    J_.clear();
    weight_.clear();

    if(!is_precomputed_) {
        cv::Mat& reference_img = reference_->frame()->level(current_level_);
        PointCloud& pointcloud_ref = reference_->pointcloud();

        precompute_patches(reference_img, pointcloud_ref, ref_patch_buf_, true);

        is_precomputed_ = true;
    }

    cv::Mat& current_img = current_->level(current_level_);
    PointCloud pointcloud_cur;
    pcl::transformPointCloud(reference_->pointcloud(), pointcloud_cur, transformation.matrix());

    precompute_patches(current_img, pointcloud_cur, cur_patch_buf_, false);

    cv::Mat errors = cv::Mat(pointcloud_cur.size(), pattern_length_, CV_32F);
//    errors = cur_patch_buf_ - (affine_a_ * ref_patch_buf_ + affine_b_);
    errors = cur_patch_buf_ - ref_patch_buf_;

    scale_ = scale_estimator_->compute(errors);

    float chi2 = 0.0f;
    size_t n_measurement = 0;

    float* errors_ptr = errors.ptr<float>();
    float* ref_patch_buf_ptr = ref_patch_buf_.ptr<float>();
    float* cur_patch_buf_ptr = cur_patch_buf_.ptr<float>();

    float IiIj = 0.0f;
    float IiIi = 0.0f;
    float sum_Ii = 0.0f;
    float sum_Ij = 0.0f;

    for(int i=0; i<errors.size().area(); ++i, ++errors_ptr, ++ref_patch_buf_ptr, ++cur_patch_buf_ptr) {

        float& res = *errors_ptr;

        float& Ii = *ref_patch_buf_ptr;
        float& Ij = *cur_patch_buf_ptr;

//        if(std::isfinite(error)  && fabs(error) < 1.0f ) {
        if(std::isfinite(res)) {
//            errors_.push_back(res);

            n_measurement++;

            Vector6 J(jacobian_buf_.col(i));

            errors_.push_back(res);
            J_.push_back(J);

            IiIj += Ii*Ij;
            IiIi += Ii*Ii;
            sum_Ii += Ii;
            sum_Ij += Ij;
//            float weight = 1.0;
//            weight = weight_function_->weight(res/scale_);
//            weight_.push_back(weight);
//            chi2 += res*res*weight;

        }

    }

    affine_a_ = IiIj / IiIi;
    affine_b_ = (sum_Ij - affine_a_*sum_Ii) / n_measurement;

//    cerr << affine_a_ << ", " << affine_b_ << endl;

    vector<float> sorted_errors;
    sorted_errors.resize(errors_.size());
    copy(errors_.begin(), errors_.end(), sorted_errors.begin());
    sort(sorted_errors.begin(), sorted_errors.end());

    float median_mu = sorted_errors[sorted_errors.size()/2];

    vector<float> absolute_res_error;
    for(auto error:errors_) {
        absolute_res_error.push_back(fabs(error-median_mu));
    }
    sort(absolute_res_error.begin(), absolute_res_error.end());
    float median_abs_deviation = 1.4826*absolute_res_error[absolute_res_error.size()/2];

//    scale_ = scale_estimator_->compute(absolute_res_error);
    for(auto error:errors_) {
        float weight=1.0;
        weight = weight_function_->weight((error-median_mu)/median_abs_deviation);
//        weight = weight_function_->weight(error/scale_);
        weight_.push_back(weight);

        chi2+= error*error*weight;
    }


//    cerr << scale_ << ", " << n_measurement << "/" << errors.size().area() << endl;
    return chi2 / n_measurement;
}

double Tracker2::build_LinearSystem(Sophus::SE3f& model)
{
    double res =  compute_residuals(model);

    H_.setZero();
    Jres_.setZero();

    for(int i=0; i < errors_.size(); ++i) {
        float& res = errors_[i];
        Vector6& J = J_[i];
        float& weight = weight_[i];

        H_.noalias() += J*J.transpose()*weight;
        Jres_.noalias() -= J*res*weight;
    }

    return res;
}

} // namespace dedvo
