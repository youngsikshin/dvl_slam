#include <dedvo/Utility.h>
#include <chrono>

namespace dedvo {

void sophus_se3f_to_eigen_isometry3f (Sophus::SE3f& src, Eigen::Isometry3f& dst)
{
    dst = src.rotationMatrix();
    dst.translation() = src.translation();
}

int64_t timestamp_now()
{
    std::chrono::system_clock::time_point now = std::chrono::high_resolution_clock::now();
    std::chrono::system_clock::duration duration_now = now.time_since_epoch();

    return std::chrono::duration_cast<std::chrono::microseconds>(duration_now).count();
}

}   // namespace dedvo
