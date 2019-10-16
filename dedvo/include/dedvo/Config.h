#pragma once
#include <iostream>
#include <string>

#include <dedvo/Datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <dedvo/CameraModel.h>
#include <dedvo/PinholeModel.h>
#include <dedvo/WeightFunction.h>

using namespace std;

namespace dedvo
{
typedef struct _camera_t camera_t;
struct _camera_t {
    int width;
    int height;

    float fx, fy, cx, cy;
    float k1, k2, p1, p2;
    float d0, d1, d2, d3, d4;

    int image_type;
};

typedef struct _camlidar_t camlidar_t;
struct _camlidar_t {
    Matrix3x4 extrinsic;
};

typedef struct _loopclosure_t loopclosure_t;
struct _loopclosure_t {
    string BoW_fname;
};

typedef struct _ORBExtractor_t ORBExtractor_t;
struct _ORBExtractor_t {
    int num_features;
    int scale_factor;
    int num_levels;
    int iniThFAST;
    int minThFAST;
};

typedef struct _tracker_t tracker_t;
struct _tracker_t {
    int levels;
    int min_level;
    int max_level;
    int max_iteration;

    bool use_weight_scale = true;
    string scale_estimator;
    string weight_function;

    ScaleEstimatorType scale_estimator_type;
    WeightFunctionType weight_function_type;

    void set_scale_estimator_type() {
        if(!scale_estimator.compare("None")) use_weight_scale = false;
        if(!scale_estimator.compare("TDistributionScale")) scale_estimator_type = ScaleEstimatorType::TDistributionScale;

        cerr << "ScaleType : " << static_cast<int> (scale_estimator_type);
    }

    void set_weight_function_type() {
        if(!weight_function.compare("TDistributionWeight")) weight_function_type = WeightFunctionType::TDistributionWeight;
    }
};

class Config
{
public:
    static Config* cfg();
    static Config* cfg(string path, string fname);
    static Config* cfg(int num_levels, int min_level, int max_level, int max_iterations);

    static string& path() { return cfg()->path_; }
    static string& fname() { return cfg()->fname_; }
    static camera_t& camera_info() { return cfg()->camera_info_; }
    static camlidar_t& camlidar() { return cfg()->camlidar_; }
    static loopclosure_t& loopclosure() { return cfg()->loopclosure_; }
    static tracker_t& tracker() { return cfg()->tracker_;}

    static CameraModel::Ptr& camera() { return cfg()->camera_; }

    static int& num_levels() { return cfg()->tracker_.levels; }
    static int& min_level() { return cfg()->tracker_.min_level; }
    static int& max_level() { return cfg()->tracker_.max_level; }

    static int& max_iterations() { return cfg()->tracker_.max_iteration; }

    void show_configuration();

private:
    Config();
    Config(string path, string fname);
    Config(int num_levels, int min_level, int max_level, int max_iterations);
    Config(const Config& other);
    ~Config() { }

    static Config* instance_;

    // configure from file
    string path_;
    string fname_;
    camera_t camera_info_;
    camlidar_t camlidar_;
    loopclosure_t loopclosure_;
    tracker_t tracker_;

    // camera model
    CameraModel::Ptr camera_;
};

}
