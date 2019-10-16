#include <dedvo/Config.h>

using namespace std;

namespace dedvo
{

Config* Config::instance_ = nullptr;

Config::Config(string path, string fname)
    :path_(path), fname_(fname)
{
    cerr << endl;
    cerr << "[Configuration]\t Load configuration from \"" << fname_ << "\"" << endl;

    cv::FileStorage f_hw_settings(path_+fname_, cv::FileStorage::READ);
    camera_info_.width = f_hw_settings["Camera.width"];
    camera_info_.height = f_hw_settings["Camera.height"];
    camera_info_.fx = f_hw_settings["Camera.fx"];
    camera_info_.fy = f_hw_settings["Camera.fy"];
    camera_info_.cx = f_hw_settings["Camera.cx"];
    camera_info_.cy = f_hw_settings["Camera.cy"];
    camera_info_.k1 = f_hw_settings["Camera.k1"];
    camera_info_.k2 = f_hw_settings["Camera.k2"];
    camera_info_.p1 = f_hw_settings["Camera.p1"];
    camera_info_.p2 = f_hw_settings["Camera.p2"];
    camera_info_.d0 = f_hw_settings["Camera.d0"];
    camera_info_.d1 = f_hw_settings["Camera.d1"];
    camera_info_.d2 = f_hw_settings["Camera.d2"];
    camera_info_.d3 = f_hw_settings["Camera.d3"];
    camera_info_.d4 = f_hw_settings["Camera.d4"];
    camera_info_.image_type = f_hw_settings["Camera.RGB"];

    cv::Mat T;
    f_hw_settings["extrinsicMatrix"] >> T;
    cv::cv2eigen(T, camlidar_.extrinsic);

    tracker_.levels = f_hw_settings["Tracker.levels"];
    tracker_.min_level = f_hw_settings["Tracker.min_level"];
    tracker_.max_level = f_hw_settings["Tracker.max_level"];
    tracker_.max_iteration = f_hw_settings["Tracker.max_iteration"];

    tracker_.scale_estimator = string(f_hw_settings["Tracker.scale_estimator"]);
    tracker_.weight_function = string(f_hw_settings["Tracker.weight_function"]);

    tracker_.set_scale_estimator_type();
    tracker_.set_weight_function_type();

    string voca_fname = string(f_hw_settings["LoopClosure.f_vocabulary"]);

    loopclosure_.BoW_fname = path_+voca_fname;
    cerr << "[CamLidarProcess]\t Set vocabulary file : " << path_+voca_fname << endl;

    camera_.reset(new PinholeModel(camera_info_.width, camera_info_.height,
                                   camera_info_.fx, camera_info_.fy, camera_info_.cx, camera_info_.cy,
                                   camera_info_.d0, camera_info_.d1, camera_info_.d2, camera_info_.d3, camera_info_.d4));

    show_configuration();
}

Config::Config()
//    :num_levels_(5), min_level_(2), max_level_(4), max_iterations_(100)
{
    show_configuration();
}

Config::Config(int num_levels, int min_level, int max_level, int max_iterations)
//    :num_levels_(num_levels), min_level_(min_level), max_level_(max_level), max_iterations_(max_iterations)
{
    show_configuration();
}

void Config::show_configuration()
{
    cerr << endl;
    cerr << "[Configuration]\t Camera information" << endl;
    cerr << "[Configuration]\t width : " << camera_info_.width << endl;
    cerr << "[Configuration]\t height : " << camera_info_.height << endl;
    cerr << "[Configuration]\t fx : " << camera_info_.fx << endl;
    cerr << "[Configuration]\t fy : " << camera_info_.fy<< endl;
    cerr << "[Configuration]\t cx : " << camera_info_.cx << endl;
    cerr << "[Configuration]\t cy : " << camera_info_.cy << endl;
    cerr << "[Configuration]\t distortion d[5] : [ " << camera_info_.d0 << ", " << camera_info_.d1 << ", " << camera_info_.d2 << ", " << camera_info_.d3 << ", " << camera_info_.d4 << "]" << endl;

    cerr << endl;
    cerr << "[Configuration]\t camera-lidar information" << endl;
    cerr << camlidar_.extrinsic.matrix() << endl;

    cerr << endl;
    cerr << "[Configuration]\t Tracker information" << endl;
    cerr << "[Configuration]\t The number of pyramid level: " << tracker_.levels << endl;
    cerr << "[Configuration]\t The minimum pyramid level: " << tracker_.min_level << endl;
    cerr << "[Configuration]\t The maximum pyramid level: " << tracker_.max_level << endl;
    cerr << "[Configuration]\t The maximum pyramid level: " << tracker_.max_iteration << endl;

    string tmp_weight_scale = (tracker_.use_weight_scale) ? "true" : "false";
    cerr << "[Configuration]\t Use weight scale: " << tmp_weight_scale << endl;
    cerr << "[Configuration]\t Scale Estimator: " << tracker_.scale_estimator << endl;
    cerr << "[Configuration]\t Weight Function: " << tracker_.weight_function << endl;

    cerr << endl;
    cerr << "[Configuration]\t LoopClosure information" << endl;
    cerr << "[Configuration]\t Vocaburaly file : " << loopclosure_.BoW_fname << endl;

}

Config* Config::cfg()
{
    if(instance_ == NULL) instance_ = new Config();

    return instance_;
}

Config* Config::cfg(string path, string fname)
{
    if(instance_ == NULL) instance_ = new Config(path, fname);

    return instance_;
}

Config* Config::cfg(int num_levels, int min_level, int max_level, int max_iterations)
{
    if(instance_ == NULL) instance_ = new Config(num_levels, min_level, max_level, max_iterations);

    return instance_;
}

}
