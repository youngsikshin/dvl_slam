# include <dedvo_gui/CamLidarProcess.h>

CamLidarProcess::CamLidarProcess(QObject *parent, QMutex *th_mutex,
                                 shared_ptr<ros::NodeHandle> node,
                                 shared_ptr<ros::NodeHandle> privante_nh)
  : QThread(parent), mutex(th_mutex), camlidar_queue_()
{
    std::string pkg_path = ros::package::getPath("dedvo_gui");
    std::string params_path = pkg_path+"/params/";
    cerr << "[CamLidarProcess]\t Parameter Path is " << params_path << endl;

//        cv::FileStorage f_ros_settings(params_path+"kitti_benchmark.yaml", cv::FileStorage::READ);
//        cv::FileStorage f_ros_settings(params_path+"tum_benchmark.yaml", cv::FileStorage::READ);
    cv::FileStorage f_ros_settings(params_path+"camlidar_system2.yaml", cv::FileStorage::READ);

    camlidar_topic_t camlidar_topic;
    camlidar_topic.cam_topic = string(f_ros_settings["Ros.Camera"]);
    camlidar_topic.lidar_topic = string(f_ros_settings["Ros.Velodyne"]);

    cerr << "[CamLidarProcess]\t Camera topic name : " << camlidar_topic.cam_topic << endl;
    cerr << "[CamLidarProcess]\t Velodyne topic name : " << camlidar_topic.lidar_topic << endl;

    string param_fname = string(f_ros_settings["Parameter.File"]);
    cerr << "[CamLidarProcess]\t parameter files : " << params_path+param_fname << endl;

    // System Setting
    dedvo_system_.reset(new dedvo::System (params_path, param_fname));
    extrinsics_ = dedvo::Config::cfg()->camlidar().extrinsic;
    camera_ = dedvo::Config::cfg()->camera();

    static_pointer_cast<dedvo::PinholeModel> (camera_)->show_intrinsic();

    camera_ = dedvo_system_->camera();
    vocabulary_ = dedvo_system_->vocabulary();

    camlidar_queue_ = shared_ptr<cam_lidar_queue_t> (new cam_lidar_queue_t);
    ros_client_ = new RosClient(node, privante_nh, camlidar_topic, camlidar_queue_);
    status_ = true;

    pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    keyframeDB_.reset(new dedvo::KeyframeDB(vocabulary_));
    frameDB_.reset(new dedvo::FrameDB());
}

CamLidarProcess::~CamLidarProcess()
{
    dedvo_system_.reset();
    this->quit();
}

void CamLidarProcess::prepare_cam_lidar() {
    while(status_) {
//        cerr << "[CamLidarProcess]\t preparing data" << endl;

//        std::unique_lock<std::mutex> ul(camlidar_queue_->mtx_camlidar);

        std::unique_lock<std::mutex> ul_pc(camlidar_queue_->mtx_pc);

        while (camlidar_queue_->pc_queue.empty()) {
            camlidar_queue_->cond_pc.wait(ul_pc);
        }

        pc_ptr_ = camlidar_queue_->pc_queue.front();


        std::unique_lock<std::mutex> ul_img(camlidar_queue_->mtx_img);

        while (camlidar_queue_->img_queue.empty()) {
            camlidar_queue_->cond_img.wait(ul_img);
        }

        if(camlidar_queue_->img_queue.size() < 2) {
//            cerr << "[CamLidarProcess]\t img_queue size < 2 " << endl;
//            ul_pc.unlock();
//            ul_img.unlock();
//            continue;
        }

        int64_t pc_timestamp = pc_ptr_->header.stamp.toNSec();

        auto last_img = camlidar_queue_->img_queue.end()-1;
        int64_t last_img_timestamp = (*last_img)->header.stamp.toNSec();

        if( last_img_timestamp < pc_timestamp) {
//            cerr << "[CamlidarProcess]\t img_timestamp < pc_timestamp" << endl;
//            cerr << last_img_timestamp << " , " << pc_timestamp << endl;
//            cerr << cnt_++ << endl;

//            ul_pc.unlock();
//            ul_img.unlock();
//            continue;
        }

        int match_idx = 0;
        int64_t min_diff = 1000000000;
//        int64_t min_diff_thresh = 156650000; //12500000;
        int64_t min_diff_thresh = 226650000;
        bool exist_match = false;

        for(auto iter = camlidar_queue_->img_queue.begin(); iter!=camlidar_queue_->img_queue.end(); ++iter, ++match_idx) {

            int64_t diff = pc_ptr_->header.stamp.toNSec() - (*iter)->header.stamp.toNSec();

            if(abs(diff) < min_diff) {
                min_diff = abs(diff);
                img_ptr_ = *iter;
            }

        }

        if (min_diff < min_diff_thresh) {
            exist_match = true;
            for(int i = 0; i < match_idx; ++i)  camlidar_queue_->img_queue.pop();

            camlidar_queue_->pc_queue.pop();

        }

        ul_pc.unlock();
        ul_img.unlock();


        if(exist_match) {
//            cerr << pc_ptr_->header.stamp.toNSec() << " - " << img_ptr_->header.stamp.toNSec() << " = " << static_cast<double> (min_diff) * 1e-9 << endl;

            std::unique_lock<std::mutex> ul_camlidar(mtx_camlidar_);
            CamlidarPair camlidar_pair = make_pair(img_ptr_, pc_ptr_);
            camlidar_pair_queue_.push(camlidar_pair);
            ul_camlidar.unlock();
            cond_camlidar_.notify_one();

        }

    }

    cerr << "[CamLidarProcess]\t Out of preparing data" << endl;
}

void CamLidarProcess::track_camlidar() {
    while(status_) {
//        cerr << "[CamLidarProcess]\t track_camlidar()" << endl;

        bool data_ready=false;
        CamlidarPair camlidar_pair;
        sensor_msgs::ImageConstPtr img_ptr;
        sensor_msgs::PointCloud2ConstPtr pc_ptr;


        if(!data_ready) {
            std::unique_lock<std::mutex> ul(mtx_camlidar_);

            while(camlidar_pair_queue_.empty()) {
//                ul.unlock();
//                continue;
                cond_camlidar_.wait(ul);
            }

            camlidar_pair = camlidar_pair_queue_.front();
            camlidar_pair_queue_.pop();

            ul.unlock();
            data_ready = true;
        }

        if(data_ready) {
            img_ptr = camlidar_pair.first;
            pc_ptr = camlidar_pair.second;


//            ofstream WriteFile;
//            WriteFile.open("frame_stamp.csv",ios_base::app);
//            WriteFile << img_ptr->header.stamp << endl;
//            WriteFile.close();

            img_ = cv_bridge::toCvCopy(img_ptr, img_ptr->encoding);
            pcl::fromROSMsg(*pc_ptr, *pc_);

            dedvo::PointCloud pointcloud;
            pcl::copyPointCloud(*pc_, pointcloud);

//            for(auto iter=pointcloud.begin(); iter != pointcloud.end(); ) {
//                double elevation = atan2(static_cast<double> (iter->z), static_cast<double> (sqrt(iter->x*iter->x+iter->y*iter->y)));
//                int i_elevation = static_cast<int> (elevation*180.0/M_PI+0.5);

//                if ( !(i_elevation == 15 || i_elevation == 11 || i_elevation == 7 || i_elevation == 3 || i_elevation == -2 || i_elevation == -6 || i_elevation == -10 || i_elevation == -14) ) {
////                if ( !(i_elevation == 15 || i_elevation == 7 || i_elevation == -2 || i_elevation == -14) ) {
////                if ( !(i_elevation == -12 || i_elevation == -14) ) {
//                    iter = pointcloud.erase(iter);
//                }
//                else {
//                    ++iter;
//                }

////                cerr << "elevation : " << i_elevation << endl;
//            }

            Eigen::Matrix4f extrinsic;
            extrinsic.block<3, 4>(0, 0) = extrinsics_;
            pcl::transformPointCloud(pointcloud, pointcloud, extrinsic);

            reference_ = current_;

            cv::Mat rgb, rectified;
//            cerr << img_->image.type() << endl;
            rgb=img_->image;  // for color kitti
//            img_->image.copyTo(rectified);  // for kitti

//            cvtColor(img_->image, rgb, CV_RGB2BGR);
            cvtColor(img_->image, rgb, CV_BayerRG2BGR);


//            camera_->undistort_image(rgb, rectified);

            rectified = rgb;
//            cvtColor(img_->image, rgb, CV_BayerBG2BGR);

//            cerr << "[CamLidarProcess]\t Initialize Frame" << endl;
            current_.reset( new dedvo::Frame(pc_ptr_->header.stamp.toNSec(), rectified, pointcloud, camera_) );
            cout << setprecision(20) << pc_ptr_->header.stamp.toSec() << endl;
//            cerr << "[CamLidarProcess]\t Set Frame" << endl;

            auto tic = ros::Time::now();
            if (dedvo_system_->track_camlidar(current_)) {
                keyframeDB_ = dedvo_system_->keyframeDB();
                frameDB_ = dedvo_system_->frameDB();
                emit callback_KF();
            }
            auto toc = ros::Time::now();

            cerr << "[CamLidarProcess]\t tracking time and rate : " << (toc-tic).toSec() << ", " << 1/((toc-tic).toSec()) << endl;



        }

        usleep(10);
    }
}

void CamLidarProcess::run() {
    ros_client_thread_ = new std::thread(&RosClient::run, ros_client_);
    camlidar_process_thread_ = new std::thread(&CamLidarProcess::prepare_cam_lidar, this);
    system_thread_ = new std::thread(&CamLidarProcess::track_camlidar, this);
}
