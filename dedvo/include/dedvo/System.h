#pragma once

#include <opencv/cv.h>
#include "Config.h"
#include "PinholeModel.h"
#include "Tracker.h"
#include "Datatypes.h"
// #include "GraphOptimizer.h"
// #include "param-util/param_util.h"
#include "Keyframe.h"
#include "KeyframeDB.h"
#include "LocalTracker.h"
#include "WindowOptimizer.h"
#include "Utility.h"
#include "ORBVocabulary.h"
#include "LoopClosing.h"
#include <dedvo/KeyframeWindow.h>
#include <dedvo/Tracker2.h>
#include <dedvo/WindowOptimizer2.h>
#include <sophus/se3.hpp>

using namespace std;

namespace dedvo {

class CameraModel;

// should be done as singleton
class UniqueId
{
public:
    UniqueId():unique_id(0)
    {

    }
    int id()
    {
      return unique_id++;
    }
private:
    int unique_id;
};

static UniqueId id_manager;

class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<System> Ptr;

    System();
    System(string path, string fname);

    ~System();
    CameraModel::Ptr camera() { return camera_; }
    KeyframeDB::Ptr keyframeDB() { return keyframeDB_; }
    FrameDB::Ptr frameDB() { return frameDB_; }
    shared_ptr<ORBVocabulary> vocabulary() { return vocabulary_; }

    // bool track_camlidar(de_dvo2::Frame::Ptr referen2ce, de_dvo2::Frame::Ptr current);
    bool track_camlidar(Frame::Ptr current);

private:

    Matrix3x4 extrinsics_;
    CameraModel::Ptr camera_;

    shared_ptr<ORBVocabulary> vocabulary_;

    KeyframeDB::Ptr keyframeDB_;
    FrameDB::Ptr frameDB_;

    Tracker::Ptr tracker_;

    // Tracker2 !!!
    Tracker2::Ptr tracker2_;
    WindowOptimizer2::Ptr window_optimizer_;

    // KF Window
    KeyframeWindow::Ptr kf_window_;

    bool tracking_status_;

    bool initialize_;
    
    Sophus::SE3f Tij_;

    Sophus::SE3f Tji_;
    Sophus::SE3f dTji_;

    int num_keyframe_;

    // WindowOptimizer::Ptr window_optimizer_;
    // GraphOptimizer graph_optimizer_;

    LoopClosing::Ptr loop_closing_;

    std::thread *loop_thread_;

    GraphOptimizer graph_optimizer_;

    bool is_prev_info = false;
    Matrix6x6 prev_info_;
};

}
