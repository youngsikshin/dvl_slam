#pragma once

#include <vector>
#include <Eigen/Core>
#include <dedvo/Keyframe.h>

namespace dedvo
{



class KeyframeWindow
{
public:
    typedef shared_ptr<KeyframeWindow> Ptr;

    KeyframeWindow(int num_keyframe);
    ~KeyframeWindow();

    void add(Keyframe::Ptr keyframe);

    int size() {  return kf_window_.size(); }

    vector<Keyframe::Ptr>::iterator begin() { return kf_window_.begin(); }
    vector<Keyframe::Ptr>::iterator end() { return kf_window_.end(); }

    vector<Keyframe::Ptr>& frames() { return kf_window_; }

private:
    int num_keyframe_;
    vector<Keyframe::Ptr> kf_window_;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H_;
};

} // namespace dedvo
