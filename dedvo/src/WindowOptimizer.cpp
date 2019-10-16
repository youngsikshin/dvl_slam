#include <dedvo/WindowOptimizer.h>

namespace dedvo {

WindowOptimizer::WindowOptimizer(CameraModel::Ptr camera, vector<Keyframe::Ptr> keyframe_window)
    :keyframe_window_(keyframe_window), iter_(0), max_iteration_(20), use_weight_scale_(true), scale_(1.0), huber_k_(1.345), dof_(5.0), pyramid_level_(1), stop_(false), n_measurement_(0), chi2_(0.0)
{
    num_keyframe_ = keyframe_window_.size();

    camera_ = camera;
    pinhole_model_ = static_pointer_cast<dedvo::PinholeModel> (camera_);

    scale_sets_.resize(num_keyframe_,num_keyframe_);

    H_.resize(6*num_keyframe_, 6*num_keyframe_);
    Jres_.resize(6*num_keyframe_, 1);
    x_.resize(6*num_keyframe_, 1);

    H_.setZero();
    Jres_.setZero();

    // keyframe_window[1]->frame()->Twc(keyframe_window[0]->frame()->Twc());

    // cerr << keyframe_window[1]->frame()->Twc().matrix() << endl;

    if(use_weight_scale_)
        weight_function_ = TDistribution;
    else
        weight_function_ = Huber;

    scale_sets_.setOnes();
    is_add_residuals_ = true;
}

WindowOptimizer::~WindowOptimizer()
{

}

void WindowOptimizer::optimize()
{
    vector<Sophus::SE3f> old_T;
    old_T.reserve(num_keyframe_);

    for(int i=0; i<num_keyframe_; i++) {
        old_T[i] = keyframe_window_[i]->frame()->Twc();
    }

    stop_ = false;

    cerr << "[WindowOptimizer]\tStart Optimizer!" << endl;
    for (iter_ = 0; iter_ < max_iteration_; ++iter_) {

        H_.setZero();
        Jres_.setZero();

        n_measurement_ = 0;
        chi2_ = 0.0;

        build_LinearSystem();        

        double new_residual = chi2_ / n_measurement_;

        if(!solve()) {
            stop_ = true;
            cerr << x_.array().isNaN() << endl;
            cerr << "[WindowOptimizer]\tNot solved!" << endl;
        }

        if( (iter_ > 0 && new_residual > residual_) || stop_) {

            for(int i=0; i<num_keyframe_; i++) {
                keyframe_window_[i]->frame()->Twc(old_T[i]);
            }

            break;
        }

        if( (iter_==0 && stop_)) {
            for(int i=0; i<num_keyframe_; i++) {
                keyframe_window_[i]->frame()->Twc(old_T[i]);
            }

            break;
        }

        for(int i=0; i<num_keyframe_; i++) {
            old_T[i] = keyframe_window_[i]->frame()->Twc();
        }

        update();

        residual_ = new_residual;

    }

    cerr << "[WindowOptimizer]\t" << iter_ << ", " << residual_ << endl;
}

void WindowOptimizer::update()
{
    // cerr << "[WindowOptimizer]\tupdate()" << endl;

    try {

        for(int i=0; i<num_keyframe_; i++) {
            Sophus::SE3f tmp = Sophus::SE3f::exp(x_.block<6,1>(6*i, 0));
            auto new_Ti = tmp*keyframe_window_[i]->frame()->Twc();
            keyframe_window_[i]->frame()->Twc(new_Ti);
        }

    }
    catch (int exception) {
        cerr << x_ << endl;
    }

}

bool WindowOptimizer::solve()
{
//     cerr << "[WindowOptimizer]\t solve()" << endl;

    // x_ = H_.ldlt().solve(Jres_);

//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> b;
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> xx;

//    A.resize(rows, rows);
//    b.resize(rows, 1);
//    xx.resize(rows, 1);

//    A.resize(6*num_keyframe_, 6*num_keyframe_);
//    b.resize(6*num_keyframe_, 1);
//    xx.resize(6*num_keyframe_, 1);

//    cerr << A.rows() << ", " << A.cols() << endl;

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

    A.block<6, 6>(0, 0) += Eigen::Matrix<double, 6, 6>::Identity(6, 6);
    A.block<6, 6>(0, 0) += Eigen::Matrix<double, 6, 6>::Identity(6, 6) * (static_cast<double>(n_measurement_));
//    A.block<6, 6>(0, 0) += Eigen::Matrix<double, 6, 6>::Identity(6, 6) * (static_cast<double>(n_measurement_)*static_cast<double>(n_measurement_))*10000000.0;

//     xx = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    xx = A.ldlt().solve(b);

    x_ = xx.cast<float> ();

    // cerr << A << endl;
    // cerr << b << endl;
    // cerr << xx << endl;



    return ((x_ - x_).array() == (x_ - x_).array()).all();
    // cerr << H_ << endl << endl;
    // cerr << x_ << endl;
}

void WindowOptimizer::build_LinearSystem()
{
    // cerr << "[WindowOptimizer]\tbuild_LinearSystem()" << endl;

    int first_n_measurement = -1;

    for(int i=0; i < num_keyframe_; ++i) {

        for(int j=0; j < num_keyframe_; ++j) {
    // for(int j=0; j < num_keyframe_; ++j) {
        // for(int i=0; i < num_keyframe_; ++i) {
            if(i == j) continue;

            Matrix12x12 H;
            Vector12 Jres;
            H.setZero();
            Jres.setZero();

            if(use_weight_scale_) {

                if(iter_ == 0) {
                    scale_ = 1.0;
                    is_add_residuals_ = false;
                    compute_residuals(keyframe_window_[i], keyframe_window_[j], H, Jres);
                    float scale = t_dist_scale(errors_);
                    scale_sets_(i,j) = scale;
                } 

                scale_ = scale_sets_(i,j);
                is_add_residuals_ = true;

            }

            H.setZero();
            Jres.setZero();
            compute_residuals(keyframe_window_[i], keyframe_window_[j], H, Jres);

            if(use_weight_scale_) {
                float scale = t_dist_scale(errors_);
                scale_sets_(i, j) = scale;
            }

            if(first_n_measurement<0) first_n_measurement = n_measurement_;
            // cerr << new_residual << endl;

            Matrix6x6 H_ii = H.block<6, 6> (0, 0);
            Matrix6x6 H_ij = H.block<6, 6> (0, 6);
            Matrix6x6 H_ji = H.block<6, 6> (6, 0);
            Matrix6x6 H_jj = H.block<6, 6> (6, 6);

            H_.block<6, 6>(6*i, 6*i) += H_ii;
            H_.block<6, 6>(6*i, 6*j) += H_ij;
            H_.block<6, 6>(6*j, 6*i) += H_ji;
            H_.block<6, 6>(6*j, 6*j) += H_jj;

            Vector6 Jres_i = Jres.block<6, 1> (0, 0);
            Vector6 Jres_j = Jres.block<6, 1> (6, 0);

            Jres_.block<6, 1> (6*i, 0) += Jres_i;
            Jres_.block<6, 1> (6*j, 0) += Jres_j;

            // cerr << H << endl;
            // cerr << Jres << endl;
            // cerr << "< " << i << ", " << j << endl;
        }
    }

    // auto tr = H_.block<6, 6>(0, 0).trace();
    // cerr << H_ << endl;
    // cerr << Jres_ << endl;

    // H_.block<6, 6>(0, 0) += Matrix6x6::Identity(6, 6) * static_cast<float> (first_n_measurement) * static_cast<float> (first_n_measurement);
    // H_.block<6, 6>(0, 0) += Matrix6x6::Identity(6, 6) * tr;
}

double WindowOptimizer::compute_residuals(Keyframe::Ptr keyframe_i, Keyframe::Ptr keyframe_j, Matrix12x12& H, Vector12& Jres)
{
    // cerr << "[WindowOptimizer]\t compute_residuals()" << endl;

    cv::Mat& img_i = keyframe_i->frame()->level(pyramid_level_);
    cv::Mat& img_j = keyframe_j->frame()->level(pyramid_level_);

    Sophus::SE3f T_i = keyframe_i->frame()->Twc();
    Sophus::SE3f T_j = keyframe_j->frame()->Twc();
    Sophus::SE3f T_ji = T_j.inverse() * T_i;

    errors_.clear();

    const int stride = img_j.cols;
    const int border = patch_halfsize_+2;
    const float scale = 1.0f/(1<<pyramid_level_);

    double chi2 = 0.0;
    int n_measurement = 0;

    cv::Mat zbuf;
    zbuf = cv::Mat::zeros(img_i.size(), CV_8U);

    for (auto iter = keyframe_i->pointcloud().begin(); iter != keyframe_i->pointcloud().end(); ++iter) {
        Eigen::Vector3f xyz_i (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_w = T_i * xyz_i;
        Eigen::Vector2f uv_i;
        uv_i.noalias() = camera_->xyz_to_uv(xyz_i) * scale;

        const float f_u_i = uv_i(0);                // float_u_i
        const float f_v_i = uv_i(1);
        const int i_u_i = static_cast<int> (f_u_i); // int_u_i
        const int i_v_i = static_cast<int> (f_v_i);

        const float subpix_u_i = f_u_i-i_u_i;
        const float subpix_v_i = f_v_i-i_v_i;
        const float w_i_tl = (1.0-subpix_u_i) * (1.0-subpix_v_i);
        const float w_i_tr = subpix_u_i * (1.0-subpix_v_i);
        const float w_i_bl = (1.0-subpix_u_i) * subpix_v_i;
        const float w_i_br = subpix_u_i * subpix_v_i;

        if (zbuf.at<uint8_t>(i_v_i, i_u_i) == 1) continue;

        zbuf.at<uint8_t>(i_v_i, i_u_i) = 1;


        Eigen::Vector3f xyz_j = T_ji * xyz_i;
        Eigen::Vector2f uv_j;
        uv_j.noalias() = camera_->xyz_to_uv(xyz_j) * scale;

        const float f_u_j = uv_j(0);
        const float f_v_j = uv_j(1);
        const int i_u_j = static_cast<int> (f_u_j);
        const int i_v_j = static_cast<int> (f_v_j);

        if ( (i_u_j - border) < 0 || (i_u_j + border) > img_i.cols || (i_v_j - border) < 0 || (i_v_j + border) > img_i.rows || xyz_j(2) <= 0)
            continue;

        const float subpix_u_j = f_u_j-i_u_j;
        const float subpix_v_j = f_v_j-i_v_j;
        const float w_j_tl = (1.0-subpix_u_j) * (1.0-subpix_v_j);
        const float w_j_tr = subpix_u_j * (1.0-subpix_v_j);
        const float w_j_bl = (1.0-subpix_u_j) * subpix_v_j;
        const float w_j_br = subpix_u_j * subpix_v_j;

        for (int y=0; y<patch_size_; ++y) {

            float *img_i_ptr = (float*) img_i.data + (i_v_i+y - patch_halfsize_)*stride + (i_u_i - patch_halfsize_);
            float *img_j_ptr = (float*) img_j.data + (i_v_j+y - patch_halfsize_)*stride + (i_u_j - patch_halfsize_);

            for (int x=0; x<patch_size_; ++x) {

                float intensity_i = w_i_tl * img_i_ptr[0] + w_i_tr * img_i_ptr[1] + w_i_bl * img_i_ptr[stride] + w_i_br * img_i_ptr[stride+1];
                float intensity_j = w_j_tl * img_j_ptr[0] + w_j_tr * img_j_ptr[1] + w_j_bl * img_j_ptr[stride] + w_j_br * img_j_ptr[stride+1];

                float res = (intensity_j - intensity_i);//*255;

                float weight = 1.0;

                if(use_weight_scale_)
                    weight = t_dist_weight(res/scale_);
                else
                    weight = huber_weight(res);

                errors_.push_back(res);

                chi2 += res*res*weight;
                n_measurement++;

                // Calculate Jacobian
                float dx = 0.5f * ((w_j_tl*img_j_ptr[1] + w_j_tr*img_j_ptr[2] + w_j_bl*img_j_ptr[stride+1] + w_j_br*img_j_ptr[stride+2])
                          -(w_j_tl*img_j_ptr[-1] + w_j_tr*img_j_ptr[0] + w_j_bl*img_j_ptr[stride-1] + w_j_br*img_j_ptr[stride]));
                float dy = 0.5f * ((w_j_tl*img_j_ptr[stride] + w_j_tr*img_j_ptr[1+stride] + w_j_bl*img_j_ptr[stride*2] + w_j_br*img_j_ptr[stride*2+1])
                          -(w_j_tl*img_j_ptr[-stride] + w_j_tr*img_j_ptr[1-stride] + w_j_bl*img_j_ptr[0] + w_j_br*img_j_ptr[1]));

                Matrix3x6 I_Pscew;
                I_Pscew.leftCols(3) = Eigen::Matrix3f::Identity(3,3);
                I_Pscew(0, 4) = xyz_w(2);  I_Pscew(0, 5) = -xyz_w(1);
                I_Pscew(1, 3) = -xyz_w(2);   I_Pscew(1, 5) = xyz_w(0);
                I_Pscew(2, 3) = xyz_w(1);  I_Pscew(2, 4) = -xyz_w(0);

                Matrix3x6 J_geo_ji = T_j.inverse().rotationMatrix() * I_Pscew;

                Eigen::Vector3f xyz_j_hat = T_j.inverse() * xyz_w;

                float z_j_hat = 1/xyz_j_hat(2);
                float z_j_hat_2 = z_j_hat*z_j_hat;
                Matrix2x3 J_proj;
                J_proj << z_j_hat, 0.0, - xyz_j_hat(0) * z_j_hat_2,
                          0.0, z_j_hat, - xyz_j_hat(1) * z_j_hat_2;

                Matrix1x2 grad_f;
                // grad_f << 255*dx*pinhole_model_->fx(), 255*dy*pinhole_model_->fy();
                grad_f << dx*pinhole_model_->fx(), dy*pinhole_model_->fy();

                Matrix1x6 Aij;

                Aij.noalias() = scale * grad_f * J_proj * J_geo_ji;
                Matrix1x6 Bij = -Aij;

                Matrix1x12 J;
                J.block<1, 6>(0, 0) = Aij;
                J.block<1, 6>(0, 6) = Bij;

                H += J.transpose() * J * weight;

                Jres -= J.transpose() * res * weight;

            }

        }
    }

    // H_ = H_/n_measurement;
    // Jres_ = Jres_/n_measurement;

    // cerr << H << endl << endl;

    // cerr << H_.determinant() << endl << endl << endl;
    // H.block<6, 6>(0, 0) += Matrix6x6::Identity(6, 6);
    // cerr << H_.determinant() << endl << endl << endl;
    // Eigen::FullPivLU<Matrix12x12> lu_decomp(H_);
    // cerr << lu_decomp.rank() << endl << endl;
    // auto x = H.ldlt().solve(Jres);

    // cerr << x << endl << endl;

    // Sophus::SE3f tmp = Sophus::SE3f::exp(x.block<6,1>(0, 0));
    // Sophus::SE3f tmp2 = Sophus::SE3f::exp(x.block<6,1>(6, 0));
    // cerr << tmp.matrix() << endl << endl;
    // cerr << tmp2.matrix() << endl;

    if(is_add_residuals_) {

        chi2_ += chi2;
        n_measurement_ += n_measurement;
        
    }
    

    return chi2/n_measurement;
}

}   // namespace dedvo
