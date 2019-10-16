#include <dedvo/WindowOptimizer2.h>

namespace dedvo
{

WindowOptimizer2::WindowOptimizer2(KeyframeWindow::Ptr kf_window)
    : iter_(0), max_iteration_(100), stop_(false), eps_(1e-10), n_measurement_(0), chi2_(0.0f)
{
//    num_keyframe_ = kf_window_->size();
    kf_window_ = kf_window;

    camera_ = Config::cfg()->camera();
    pinhole_model_ = static_pointer_cast<dedvo::PinholeModel> (camera_);

    current_level_ = 0;

    auto tracker_info = Config::cfg()->tracker();

    use_weight_scale_ = tracker_info.use_weight_scale;

    set_weightfunction();
}

WindowOptimizer2::~WindowOptimizer2()
{

}

void WindowOptimizer2::set_weightfunction()
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

//    weight_function_.reset(new HuberWeightFunction());

}

bool WindowOptimizer2::refine()
{
  cerr << "[WindowOptimizer2]\t called refine()" << endl;
  num_keyframe_ = kf_window_->size();

  vector<Sophus::SE3f> old_T;
  old_T.reserve(num_keyframe_);

  auto kf_window = kf_window_->frames();
  for(int i=0; i<num_keyframe_; i++) {
      old_T[i] = kf_window[i]->frame()->Twc();
  }

  H_.resize(6*num_keyframe_, 6*num_keyframe_);
  Jres_.resize(6*num_keyframe_, 1);
  x_.resize(6*num_keyframe_, 1);

  stop_ = false;

  for (iter_ = 0; iter_ < max_iteration_; ++iter_) {

      H_.setZero(6*num_keyframe_, 6*num_keyframe_);
      Jres_.setZero(6*num_keyframe_, 1);
      n_measurement_ = 0;
      chi2_ = 0.0f;

      build_LinearSystem();

      double new_residual = chi2_ / n_measurement_;

      if(!solve()) {
          stop_ = true;
          cerr << x_.array().isNaN() << endl;
      }

      if( (iter_ > 0 && new_residual > residual_) || stop_) {

          for(int i=0; i<num_keyframe_; i++) {
              kf_window[i]->frame()->Twc(old_T[i]);
          }

          break;
      }

      if( (iter_==0 && stop_)) {
          for(int i=0; i<num_keyframe_; i++) {
              kf_window[i]->frame()->Twc(old_T[i]);
          }

          break;
      }

      update();

      for(int i=0; i<num_keyframe_; i++) {
          old_T[i] = kf_window[i]->frame()->Twc();
      }

      residual_ = new_residual;

      if ( ((x_ - x_).array() != (x_ - x_).array()).all() ) {
//          status_ = false;
          break;
      }
      else {
          double max = -1;
          for (int i=0; i<x_.size(); i++)
          {
            double abs = fabs(x_(i,0));
            if(abs>max){
              max = abs;
            }
          }

          if( max < eps_ ) {
              break;
          }
      }

  }

  cerr << "[WindowOptimizer2]\t" << iter_ << endl;

}

void WindowOptimizer2::update()
{
    // cerr << "[WindowOptimizer]\tupdate()" << endl;

    try {
        auto kf_window = kf_window_->frames();

        for(int i=0; i<num_keyframe_; i++) {
            Sophus::SE3f tmp = Sophus::SE3f::exp(x_.block<6,1>(6*i, 0));
            auto new_Ti = tmp*kf_window[i]->frame()->Twc();
            kf_window[i]->frame()->Twc(new_Ti);
        }

    }
    catch (int exception) {
        cerr << x_ << endl;
    }

}


bool WindowOptimizer2::solve()
{

     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
     Eigen::Matrix<double, Eigen::Dynamic, 1> b;
     Eigen::Matrix<double, Eigen::Dynamic, 1> xx;

     A.resize(6*num_keyframe_, 6*num_keyframe_);
     b.resize(6*num_keyframe_, 1);
     xx.resize(6*num_keyframe_, 1);

//    A = H_.cast <double> () / (static_cast<double>(n_measurement_)*static_cast<double>(n_measurement_));   // Matrix of floats.
//    b = Jres_.cast <double> () / (static_cast<double>(n_measurement_)*static_cast<double>(n_measurement_));
     A = H_.cast <double> ();
     b = Jres_.cast <double> ();

//    A.block<6, 6>(0, 0) += Eigen::Matrix<double, 6, 6>::Identity(6, 6);
//    A.block<6, 6>(0, 0) += Eigen::Matrix<double, 6, 6>::Identity(6, 6) * (static_cast<double>(n_measurement_));
    A.block<6, 6>(0, 0) += Eigen::Matrix<double, 6, 6>::Identity(6, 6) * (static_cast<double>(n_measurement_)*static_cast<double>(n_measurement_))*10000000.0;

//     xx = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    xx = A.ldlt().solve(b);

    x_ = xx.cast<float> ();


    return ((x_ - x_).array() == (x_ - x_).array()).all();

}
//void WindowOptimizer2::update (const ModelType& old_model, ModelType& new_model)
//{
////    new_model = old_model * Sophus::SE3f::exp(-x_);\

////    cerr << "[Tracker2]\t The model was updated." << endl;
////    cerr << new_model.matrix() << endl << endl;
//}

void WindowOptimizer2::precompute_patches(cv::Mat& img, PointCloud& pointcloud, cv::Mat& patch_buf, Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>& jacobian_buf, bool is_derivative)
{
    const int border = patch_halfsize_+2+2;
    const int stride = img.cols;
    const float scale = 1.0f/(1<<current_level_);

    cv::Mat zbuf;
    zbuf = cv::Mat::zeros(img.size(), CV_8U);

    vector<Eigen::Vector2f> uv_set = camera_->pointcloud_to_uv(pointcloud, scale);

    patch_buf = cv::Mat(pointcloud.size(), pattern_length_, CV_32F);

//    if(is_derivative) {
        Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf;
        dI_buf.resize(Eigen::NoChange, patch_buf.rows*pattern_length_);
        dI_buf.setZero(2, patch_buf.rows*pattern_length_);

        jacobian_buf.resize(Eigen::NoChange, patch_buf.rows*pattern_length_);

        jacobian_buf.setZero(3, patch_buf.rows*pattern_length_);
//    }

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

//            if(is_derivative) {
                // precompute image gradient
                float dx = 0.5f * ((w_tl*img_ptr[1] + w_tr*img_ptr[2] + w_bl*img_ptr[stride+1] + w_br*img_ptr[stride+2])
                          -(w_tl*img_ptr[-1] + w_tr*img_ptr[0] + w_bl*img_ptr[stride-1] + w_br*img_ptr[stride]));
                float dy = 0.5f * ((w_tl*img_ptr[stride] + w_tr*img_ptr[1+stride] + w_bl*img_ptr[stride*2] + w_br*img_ptr[stride*2+1])
                          -(w_tl*img_ptr[-stride] + w_tr*img_ptr[1-stride] + w_bl*img_ptr[0] + w_br*img_ptr[1]));

                Matrix2x6 frame_jac;
                Eigen::Vector3f xyz(pc_iter->x, pc_iter->y, pc_iter->z);
//                Frame::jacobian_xyz2uv(xyz, frame_jac);
                Matrix1x3 frame_jacobian;
                auto fx = pinhole_model_->fx();
                auto fy = pinhole_model_->fy();

                frame_jacobian << dx*fx/pc_iter->z,
                                  dy*fy/pc_iter->z,
                                  -1*dx*fx*pc_iter->x/(pc_iter->z*pc_iter->z)-1*dy*fy*pc_iter->y/(pc_iter->z*pc_iter->z);

                jacobian_buf.col(point_counter*pattern_length_ + i) = frame_jacobian;
//            }

        }
    }

}

void WindowOptimizer2::compute_residuals(Keyframe::Ptr keyframe_m, Keyframe::Ptr keyframe_n,
                                           vector<float>& residuals,
                                           vector<Matrix1x6>& frame_jacobian_m, vector<Matrix1x6>& frame_jacobian_n)
{
//    cerr << "[WindowOptimizer2]\t Called compute_residuals between frame " << keyframe_m->id() << " and frame " << keyframe_n->id() << endl;

    cv::Mat img_n = keyframe_n->frame()->level(current_level_);
    PointCloud& pointcloud_n = keyframe_n->pointcloud();

    cv::Mat patch_buf_n;
//    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf_m;
    Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf_n;

    precompute_patches(img_n, pointcloud_n, patch_buf_n, jacobian_buf_n, false);

    Sophus::SE3f T_m = keyframe_m->frame()->Twc();
    Sophus::SE3f T_n = keyframe_n->frame()->Twc();
    Sophus::SE3f T_mn = T_m.inverse() * T_n;

    cv::Mat img_m = keyframe_m->frame()->level(current_level_);
    PointCloud pointcloud_m;

    pcl::transformPointCloud(pointcloud_n, pointcloud_m, T_mn.matrix());

    cv::Mat patch_buf_m;
//    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf_n;
    Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf_m;

    precompute_patches(img_m, pointcloud_m, patch_buf_m, jacobian_buf_m, false);

    cv::Mat errors = patch_buf_m - patch_buf_n;


    float* errors_ptr = errors.ptr<float>();
    float* patch_buf_m_ptr = patch_buf_m.ptr<float>();
    float* patch_buf_n_ptr = patch_buf_n.ptr<float>();

//    int m = keyframe_m->id();
//    int n = keyframe_n->id();


    int k = 0;    // point n idx

//    for(int k=0; k<pointcloud_n.size(); ++k) {
    for(auto iter = pointcloud_n.begin(); iter != pointcloud_n.end(); ++iter, ++k) {
        for(int i=0; i<pattern_length_; ++i) {
            int buf_idx = k*pattern_length_+i;

            float& res = *(errors_ptr + buf_idx);

            if(std::isfinite(res)) {
                //TODO: calc errors, jacobian, a(k), weight
                residuals.push_back(res);

                Eigen::Vector3f xyz_n (iter->x, iter->y, iter->z);
                Eigen::Vector3f xyz_wn = T_n * xyz_n;

                Matrix3x6 I_Pscew;
                I_Pscew.leftCols(3) = Eigen::Matrix3f::Identity(3,3);
                I_Pscew(0, 4) = xyz_wn(2);  I_Pscew(0, 5) = -xyz_wn(1);
                I_Pscew(1, 3) = -xyz_wn(2);   I_Pscew(1, 5) = xyz_wn(0);
                I_Pscew(2, 3) = xyz_wn(1);  I_Pscew(2, 4) = -xyz_wn(0);

                Matrix3x6 J_point = T_m.inverse().rotationMatrix() * I_Pscew;

                frame_jacobian_m.push_back(-1.0*jacobian_buf_m.col(buf_idx).transpose()*J_point);
                frame_jacobian_n.push_back(jacobian_buf_m.col(buf_idx).transpose()*J_point);

//                frame_jacobian_m.push_back(jacobian_buf_m.col(buf_idx));
//                frame_jacobian_n.push_back(jacobian_buf_n.col(buf_idx));

//                pair<Keyframe::Ptr, int> anchor_idx = make_pair(keyframe_m, k);
//                anchors_idx.push_back(anchor_idx);
            }

        }
    }

//    for(int k=0; k<errors.size().area(); ++k, ++errors_ptr, ++patch_buf_m_ptr, ++patch_buf_n_ptr) {

//        float& res = *errors_ptr;

//    }
}

double WindowOptimizer2::build_LinearSystem()
{
    auto frames = kf_window_->frames();

    vector<Matrix1x6> full_jacobian_m, full_jacobian_n;
    vector<float> full_residuals;
    vector<pair<size_t, size_t>> frame_idx_map;

    for(size_t n = 0; n < frames.size(); ++n) {
        Keyframe::Ptr frame_n = frames[n];


        for(size_t m = 0; m < frames.size(); ++m) {
            if( m != n) {
                pair<size_t, size_t> frame_idx_pair = make_pair(m, n);

                Keyframe::Ptr frame_m = frames[m];

                vector<float> residuals;
                vector<Matrix1x6> frame_jacobian_m, frame_jacobian_n;
//                vector<pair<Keyframe::Ptr, int>> anchors_idx;

                compute_residuals(frame_m, frame_n, residuals, frame_jacobian_m, frame_jacobian_n);

                full_residuals.insert(full_residuals.end(), residuals.begin(), residuals.end());
                full_jacobian_m.insert(full_jacobian_m.end(), frame_jacobian_m.begin(), frame_jacobian_m.end());
                full_jacobian_n.insert(full_jacobian_n.end(), frame_jacobian_n.begin(), frame_jacobian_n.end());

                for(size_t k=0; k<residuals.size(); ++k) {
                    frame_idx_map.push_back(frame_idx_pair);
                }

            }
        }

    }

    vector<float> sorted_errors;
    sorted_errors.resize(full_residuals.size());
    copy(full_residuals.begin(), full_residuals.end(), sorted_errors.begin());
    sort(sorted_errors.begin(), sorted_errors.end());

    float median_mu = sorted_errors[sorted_errors.size()/2];

    vector<float> absolute_res_error;
    for(auto error:full_residuals) {
        absolute_res_error.push_back(fabs(error-median_mu));
    }
    sort(absolute_res_error.begin(), absolute_res_error.end());
    float median_abs_deviation = 1.4826*absolute_res_error[absolute_res_error.size()/2];

//    float scale = scale_estimator_->compute(absolute_res_error);

//    float scale = scale_estimator_->compute(full_residuals);

//    float scale = 1.0;

    vector<float> weights;

    n_measurement_ = full_residuals.size();

    // assemble linear system
    for(auto error:full_residuals) {
        float weight=1.0;
//        weight = weight_function_->weight(error/scale);
        weight = weight_function_->weight((error-median_mu)/median_abs_deviation);
        weights.push_back(weight);

        chi2_ += error*error*weight;
    }

    for(size_t k=0; k < full_residuals.size(); ++k) {
        Matrix12x12 H;
        Vector12 Jres;
        H.setZero(12, 12);
        Jres.setZero(12, 1);

        auto frame_idx_pair = frame_idx_map[k];
        size_t m = frame_idx_pair.first;
        size_t n = frame_idx_pair.second;

        float& res = full_residuals[k];
        float& weight = weights[k];
        Matrix1x6& Jm = full_jacobian_m[k];
        Matrix1x6& Jn = full_jacobian_n[k];
        Matrix1x12 J;
        J.block<1,6>(0,0) = Jm;
        J.block<1,6>(0,6) = Jn;

//        Jm.block<1,3>(0,3) = Jm.block<1,3>(0,3)*100;
//        Jn.block<1,3>(0,3) = Jn.block<1,3>(0,3)*100;

        H = J.transpose() * J *weight;
        Jres = J.transpose() * res * weight;

        Matrix6x6 H_mm = H.block<6, 6> (0, 0);
        Matrix6x6 H_mn = H.block<6, 6> (0, 6);
        Matrix6x6 H_nm = H.block<6, 6> (6, 0);
        Matrix6x6 H_nn = H.block<6, 6> (6, 6);

        H_.block<6, 6>(6*m, 6*m) += H_mm;
        H_.block<6, 6>(6*m, 6*n) += H_mn;
        H_.block<6, 6>(6*n, 6*m) += H_nm;
        H_.block<6, 6>(6*n, 6*n) += H_nn;

        Vector6 Jres_m = Jres.block<6,1> (0, 0);
        Vector6 Jres_n = Jres.block<6,1> (6, 0);

        Jres_.block<6, 1> (6*m, 0) -= Jres_m;
        Jres_.block<6, 1> (6*n, 0) -= Jres_n;
    }

    return chi2_/n_measurement_;
}

}
