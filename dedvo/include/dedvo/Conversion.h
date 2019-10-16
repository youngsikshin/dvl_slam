#pragma once
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dedvo
{

class Converter {
public:
    static void sophus_se3f_to_eigen_isometry3f (Sophus::SE3f& src, Eigen::Isometry3f& dst);
};




} // namespace dedvo
