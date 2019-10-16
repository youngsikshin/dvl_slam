#pragma once
#include <list>
#include <mutex>

#include <dedvo/Keyframe.h>
#include <dedvo/KeyframeDB.h>
#include <dedvo/GraphOptimizer.h>
#include <dedvo/Tracker.h>

using namespace std;

namespace dedvo
{

class LoopClosing
{
public:
    typedef shared_ptr<LoopClosing> Ptr;
    LoopClosing(KeyframeDB::Ptr keyframeDB, shared_ptr<ORBVocabulary> vocabulary, CameraModel::Ptr camera);
    ~LoopClosing();
    void run();
    bool run_without_thread();
    void request_finish();
    void insert_keyframe(Keyframe::Ptr keyframe);
    bool loop_constraint();

    Sophus::SE3f& Tij() { return Tij_; }
    Sophus::SE3f& Tji() { return Tji_; }
    Sophus::SE3d dTji() { return Tji_.cast<double>(); }

    Keyframe::Ptr current_keyframe() { return current_keyframe_; }
    Keyframe::Ptr loop_keyframe() { return loop_keyframe_; }

    Tracker::Ptr tracker() { return tracker_; }

    bool is_finished();

private:
    CameraModel::Ptr camera_;
    Tracker::Ptr tracker_;

    bool is_finished_;
    void finish();
//    bool is_finished();

    int last_loop_id_;
    int num_loop_;
    Keyframe::Ptr current_keyframe_;
    Keyframe::Ptr loop_keyframe_;

    Sophus::SE3f Tji_;
    Sophus::SE3f Tij_;

    list<Keyframe::Ptr> keyframe_queue_;
    std::mutex mtx_kf_queue_;

    bool check_new_keyframe();
    bool detect_loop();
    vector<Keyframe::Ptr> detect_loop_candidate(Keyframe::Ptr keyframe, float min_si);

    bool request_finish_;
    std::mutex mtx_request_finish_;
    bool check_finish();

    KeyframeDB::Ptr keyframeDB_;
    vector<Keyframe::Ptr> keyframes_;
    shared_ptr<ORBVocabulary> vocabulary_;

    GraphOptimizer::Ptr graph_optimizer_;
    void correct_loop();
};

} // namespace dedvo
