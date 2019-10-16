#include <dedvo/LocalTracker.h>

namespace dedvo {

LocalTracker::LocalTracker(CameraModel::Ptr camera)
  : min_level_(Config::cfg()->min_level()),
    max_level_(Config::cfg()->max_level()),
    current_level_(0),
    iter_(0), max_iteration_(1000), stop_(false), n_measurement_(0), residual_(-1.0), eps_(1e-10), status_(false),
    use_weight_scale_(true), scale_(1.0), huber_k_(1.345), dof_(5.0)
{
    camera_ = camera;
    pinhole_model_ = static_pointer_cast<dedvo::PinholeModel> (camera_);

    if(use_weight_scale_)
        weight_function_ = TDistribution;
    else
        weight_function_ = Huber;
}

LocalTracker::~LocalTracker()
{

}

bool LocalTracker::solve()
{
    x_ = H_.ldlt().solve(Jres_);

    // if((bool) std::isnan((double) x_[0]))
    //     return false;

    if ( ((x_ - x_).array() == (x_ - x_).array()).all() )
        return true;

    return false;
}

void LocalTracker::update(Keyframe::Ptr keyframe_j)
{
//    Sophus::SE3f Tjw = Sophus::SE3f::exp(-x_) * keyframe_j->frame()->Twc().inverse();
//    keyframe_j->frame()->Twc(Tjw.inverse());

//    cerr << " start update " << endl;
    Sophus::SE3f Twj = Sophus::SE3f::exp(-x_)*keyframe_j->frame()->Twc();
    keyframe_j->frame()->Twc(Twj);
//    cerr << " end update " << endl;
}

bool LocalTracker::tracking(vector<Keyframe::Ptr> keyframe_window)
{

    int num_keyframe = keyframe_window.size();

    Keyframe::Ptr current_keyframe = keyframe_window[num_keyframe-1];

    Sophus::SE3f old_Tj = current_keyframe->frame()->Twc();
    Matrix6x6 old_H = Matrix6x6::Identity();

    double new_residual = 0.0f;

    cerr << "[LocalTracker]\t 0" << endl;
    for( int i=0; i<num_keyframe-1; ++i) {

//        cerr << "[LocalTracker]\t 0pre " << i << endl;
        Keyframe::Ptr reference_keyframe = keyframe_window[i];

        new_residual += compute_residuals_pattern(reference_keyframe, current_keyframe);
//        cerr << "[LocalTracker]\t 1pre " << i << endl;
    }

    new_residual = new_residual / n_measurement_;
    residual_ = new_residual;

//    cerr << "[LocalTracker]\t 0-1"  << " " << residuals_.size()<< endl;
    if(use_weight_scale_) scale_ = t_dist_scale(residuals_);

    for (iter_=0; iter_<max_iteration_; ++iter_) {

//        cerr << "[LocalTracker]\t 1" << endl;
        H_.setZero();
        Jres_.setZero();

        n_measurement_ = 0;

        new_residual=0.0f;
        residuals_.clear();

//        cerr << "[LocalTracker]\t 2" << endl;
        for( int i=0; i<num_keyframe-1; ++i) {

            Keyframe::Ptr reference_keyframe = keyframe_window[i];

            new_residual += compute_residuals_pattern(reference_keyframe, current_keyframe);

        }

        new_residual = new_residual/n_measurement_;


//        cerr << "[LocalTracker]\t 3" << endl;
        if(!solve()) {
            cerr << "[LocalTracker]\t Hessian is closed to singular!" << endl;
            stop_ = true;
        }

        if( iter_ == 0 && stop_) {
            break;
        }

        if( (iter_ > 0 && new_residual > residual_) || stop_) {
            current_keyframe->frame()->Twc(old_Tj);
            H_ = old_H;
            status_ = false;
            break;
        }

        if(use_weight_scale_) scale_ = t_dist_scale(residuals_);

//        cerr << "[LocalTracker]\t 4" << endl;
        update(current_keyframe);

        old_H = H_;
        old_Tj = current_keyframe->frame()->Twc();

        residual_ = new_residual;

        if (norm_max(x_) < eps_) {
           status_ = true;

           if ( ((x_ - x_).array() != (x_ - x_).array()).all() )
               status_ = false;

           break;
        }

        cerr << "[LocalTracker]\t" << iter_ << "iteration / ";

        cerr << new_residual << ", " << residual_ << endl;


    }


}

double LocalTracker::compute_residuals_pattern(Keyframe::Ptr keyframe_i, Keyframe::Ptr keyframe_j)
{
    cv::Mat& img_i = keyframe_i->frame()->level(current_level_);
    cv::Mat& img_j = keyframe_j->frame()->level(current_level_);

    Sophus::SE3f T_i = keyframe_i->frame()->Twc();
    Sophus::SE3f T_j = keyframe_j->frame()->Twc();
    Sophus::SE3f Tji = T_j.inverse() * T_i;

    const int stride = img_j.cols;
    const float scale = 1.0f/(1<<current_level_);
    const int border = patch_halfsize_+2+2;

    double chi2 = 0.0;

    for(auto iter = keyframe_i->pointcloud().begin(); iter != keyframe_i->pointcloud().end(); ++iter) {
//        cerr << "[LocalTracker]\t res 0 " << endl;
        Eigen::Vector3f xyz_i (iter->x, iter->y, iter->z);
        Eigen::Vector3f xyz_w = T_i * xyz_i;
        Eigen::Vector2f uv_i;
        uv_i.noalias() = camera_->xyz_to_uv(xyz_i) * scale;

//        cerr << "[LocalTracker]\t res 1 " << endl;
        const float f_u_i = uv_i(0);
        const float f_v_i = uv_i(1);
        const int i_u_i = static_cast<int> (f_u_i); // int_u_i
        const int i_v_i = static_cast<int> (f_v_i);

        const float subpix_u_i = f_u_i-i_u_i;
        const float subpix_v_i = f_v_i-i_v_i;
        const float w_i_tl = (1.0-subpix_u_i) * (1.0-subpix_v_i);
        const float w_i_tr = subpix_u_i * (1.0-subpix_v_i);
        const float w_i_bl = (1.0-subpix_u_i) * subpix_v_i;
        const float w_i_br = subpix_u_i * subpix_v_i;

//        cerr << "[LocalTracker]\t res 2 " << endl;
        Eigen::Vector3f xyz_j = Tji * xyz_i;
        Eigen::Vector2f uv_j;
        uv_j.noalias() = camera_->xyz_to_uv(xyz_j) * scale;

        const float f_u_j = uv_j(0);
        const float f_v_j = uv_j(1);
        const int i_u_j = static_cast<int> (f_u_j);
        const int i_v_j = static_cast<int> (f_v_j);

//        cerr << "[LocalTracker]\t res 3 " << endl;
        if ( (i_u_j - border) < 0 || (i_u_j + border) > img_i.cols || (i_v_j - border) < 0 || (i_v_j + border) > img_i.rows || xyz_j(2) <= 0)
            continue;

        const float subpix_u_j = f_u_j-i_u_j;
        const float subpix_v_j = f_v_j-i_v_j;
        const float w_j_tl = (1.0-subpix_u_j) * (1.0-subpix_v_j);
        const float w_j_tr = subpix_u_j * (1.0-subpix_v_j);
        const float w_j_bl = (1.0-subpix_u_j) * subpix_v_j;
        const float w_j_br = subpix_u_j * subpix_v_j;

//        cerr << "[LocalTracker]\t res 4 " << endl;
        for (int i=0; i < pattern_length_; ++i) {
            int x = pattern_[i][0];
            int y = pattern_[i][1];

            float *img_i_ptr = (float*) img_i.data + (i_v_i+y)*stride + (i_u_i+x);
            float *img_j_ptr = (float*) img_j.data + (i_v_j+y)*stride + (i_u_j+x);

            float intensity_i = w_i_tl*img_i_ptr[0] + w_i_tr*img_i_ptr[1] + w_i_bl*img_i_ptr[stride] + w_i_br*img_i_ptr[stride+1];
            float intensity_j = w_j_tl*img_j_ptr[0] + w_j_tr*img_j_ptr[1] + w_j_bl*img_j_ptr[stride] + w_j_br*img_j_ptr[stride+1];

            float res = (intensity_j - intensity_i);

            float weight = 1.0f;

//            cerr << "[LocalTracker]\t res 5 " << endl;
            if(use_weight_scale_)
                weight = t_dist_weight(res/scale_);
            else
                weight = huber_weight(res);

            residuals_.push_back(res);

            chi2 += res*res*weight;

            n_measurement_++;

//            cerr << "[LocalTracker]\t res 6 " << endl;
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
//             grad_f << 255*dx*pinhole_model_->fx(), 255*dy*pinhole_model_->fy();
            grad_f << dx*pinhole_model_->fx(), dy*pinhole_model_->fy();
            Matrix1x6 J = scale * grad_f * J_proj * J_geo_ji;

//            cerr << "[LocalTracker]\t res 7 " << endl;
            // // evaluate projection jacobian
//            Matrix2x6 frame_jac;
//            Frame::jacobian_xyz2uv(xyz_j, frame_jac);
//            frame_jac = frame_jac;

//            Matrix1x6 J = ( dx* pinhole_model_->fx() * frame_jac.row(0) + dy*pinhole_model_->fy()*frame_jac.row(1) ) / (1<<current_level_);
            H_.noalias() += J.transpose()*J*weight;
            Jres_.noalias() -= J.transpose()*res*weight;


        }
    }

    return chi2;
}

} // namespace dedvo
