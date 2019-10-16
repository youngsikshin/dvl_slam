#pragma once
#include <utility>
#include "Frame.h"
#include "CameraModel.h"
#include "PinholeModel.h"

namespace dedvo {

class Keyframe {
public:
    typedef shared_ptr<Keyframe> Ptr;
    Keyframe(Frame::Ptr frame, int id_, CameraModel::Ptr camera, shared_ptr<ORBVocabulary> vocabulary);
    Keyframe(Frame::Ptr frame, CameraModel::Ptr camera, shared_ptr<ORBVocabulary> vocabulary);
    ~Keyframe();
    
    Frame::Ptr frame() {
        return frame_;
    }

    int pointcloud_size() { return pointcloud_.size(); }
//    vector<Point, Eigen::aligned_allocator<Eigen::Vector3f>>& pointcloud() { return pointcloud_; }
    PointCloud& pointcloud() { return pointcloud_; }
//    cv::Subdiv2D& subdiv() { return subdiv_; }

    void id(int id) { id_ = id; }
    int& id() { return id_; }

    void point_sampling();
    void parent(Keyframe::Ptr parent) { parent_ = parent; }
    void child(Keyframe::Ptr child) { child_ = child; }

    float get_visible_ratio (const Keyframe::Ptr keyframe);
    Keyframe::Ptr parent() { return parent_; }
    Keyframe::Ptr child() { return child_; }

    void ORB_extractor();
    void compute_BoW();

    DBoW2::BowVector& BoW_vec() { return BoW_vec_; }

    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    int& loop_query() { return loop_query_; }
    void loop_query(int loop_query) { loop_query_ = loop_query; }

    int& num_loop_word() { return num_loop_word_; }
    void num_loop_word(int num_loop_word) { num_loop_word_ = num_loop_word; }
    void inc_num_loop_word() { num_loop_word_++; }

    float& loop_score() { return loop_score_; }
    void loop_score(float loop_score) { loop_score_ = loop_score; }

    void first_connection(bool first_connection) { first_connection_ = first_connection; }
    bool first_connection() { return first_connection_; }

    void show_image_with_points(cv::Mat& img, size_t num_level);

private:
    int id_;
    Frame::Ptr frame_;
    CameraModel::Ptr camera_;
    PinholeModel::Ptr pinhole_model_;

//    vector<Point, Eigen::aligned_allocator<Eigen::Vector3f>> pointcloud_;
    PointCloud pointcloud_;

    bool first_connection_;
    Keyframe::Ptr parent_;
    Keyframe::Ptr child_;

    //Feature Extractor
//    void ORB_extractor();
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    // Bag of Words Vector structures.
//    void compute_BoW();
    shared_ptr<ORBVocabulary> vocabulary_;
    DBoW2::BowVector BoW_vec_;
    DBoW2::FeatureVector feat_vec_;

    int loop_query_;
    int num_loop_word_;
    float loop_score_;
};

}   // namespace ddslam
