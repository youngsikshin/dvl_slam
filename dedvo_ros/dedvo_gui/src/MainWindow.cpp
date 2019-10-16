#include <string>
#include <dedvo_gui/MainWindow.h>
#include "ui_MainWindow.h"

//MainWindow::MainWindow(int argc, char *argv[], QWidget *parent) :
MainWindow::MainWindow(int argc, char* argv[], QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ros::init(argc, argv, "dedvo_gui_node");
    node_.reset(new ros::NodeHandle());
    privante_nh_.reset(new ros::NodeHandle("~"));
//    camlidar_process_ = new CamLidarProcess(node_, privante_nh_);
    camlidar_process_ = new CamLidarProcess(this, &mutex, node_, privante_nh_);

//    camlidar_thread_ = new std::thread(&CamLidarProcess::run, camlidar_process_);

    connect(camlidar_process_, SIGNAL(callback_KF()), this, SLOT(set_KF()));

    connect(this->ui->actionSave_posedata, SIGNAL(triggered()), this, SLOT(save_poses()));
    connect(this->ui->actionSave_marker_position, SIGNAL(triggered()), this, SLOT(save_marker_positions()));
    connect(this->ui->actionSave_marker_poses, SIGNAL(triggered()), this, SLOT(save_marker_pose()));
    connect(this->ui->actionSave_point_map, SIGNAL(triggered()), this, SLOT(save_point_map()));

    connect(this->ui->dedvoGLWidget->timer_, SIGNAL(timeout()), this, SLOT(drawGL()));
    ui->dedvoGLWidget->timer_->start(10);

    camlidar_process_->start();
}

MainWindow::~MainWindow()
{
    ros::shutdown();
//    camlidar_process_->stop();
//    camlidar_thread_->join();
    camlidar_process_->stop();

    while(!camlidar_process_->isFinished());

    delete ui;
}

void MainWindow::drawGL()
{
//    if(ui->dedvoGLWidget->is_drawing())
        ui->dedvoGLWidget->update();
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void MainWindow::set_KF()
{
//    cerr << "set KF()" << endl;

    keyframeDB_ = camlidar_process_->keyframeDB();
    frameDB_ = camlidar_process_->frameDB();
    this->ui->dedvoGLWidget->keyframeDB( camlidar_process_->keyframeDB() );
    this->ui->dedvoGLWidget->frameDB( camlidar_process_->frameDB() );
    this->ui->dedvoGLWidget->Twc_list();
    this->ui->dedvoGLWidget->pointcloud_list();
//    this->ui->dedvoGLWidget->update();
//    this->ui->dedvoGLWidget->save_framebuffer();

    QMatrix4x4 current_pose = this->ui->dedvoGLWidget->current_pose();
//    detect_aruco(current_pose);

}

void MainWindow::detect_aruco(QMatrix4x4 current_pose)
{
    //    aruco detector
    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;

    cv::Mat latest_img = camlidar_process_->keyframeDB()->latest_keyframe()->frame()->level(0);
    cv::Mat original = camlidar_process_->keyframeDB()->latest_keyframe()->frame()->original_img();
    latest_img.convertTo(latest_img, CV_8UC1, 255);//, 1.0/255);

    cv::aruco::detectMarkers(latest_img, dictionary, corners, ids);

    cv::Mat camMatrix = cv::Mat::eye(3, 3, CV_64FC1);

    camMatrix.at< double >(0, 0) = 407.5489339719625;
    camMatrix.at< double >(1, 1) = 410.9210229027896;
    camMatrix.at< double >(0, 2) = 384.2126542768888;
    camMatrix.at< double >(1, 2) = 297.2707904584936;

    cv::Mat distCoeffs(5, 1, CV_64FC1, cv::Scalar::all(0));
    distCoeffs.at< double >(0) = -0.049204252847251;
    distCoeffs.at< double >(1) = 0.118763673090059;
    distCoeffs.at< double >(2) = 0.000633039927511910;
    distCoeffs.at< double >(3) = -0.000504103201163194;
    distCoeffs.at< double >(4) = -0.059989499814272;

    vector<cv::Vec3d> rvecs, tvecs;

    cv::Mat marker_img;
    cvtColor(latest_img, marker_img, cv::COLOR_GRAY2BGR);;
    if (ids.size() > 0) {
        for (size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] == 213 || ids[i] == 100 || ids[i] == 120 || ids[i] == 140 || ids[i] == 160 || ids[i] == 180) {
                cv::aruco::drawDetectedMarkers(marker_img, corners, ids);
                cv::aruco::estimatePoseSingleMarkers(corners, 0.205, camMatrix, distCoeffs, rvecs, tvecs);
                cv::aruco::drawAxis(marker_img, camMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);

                cv::Mat Rot;
                cv::Rodrigues(rvecs[0],Rot);

                QVector3D marker_position_c(tvecs[0](0), tvecs[0](1), tvecs[0](2));
                QVector3D marker_position_w = current_pose * marker_position_c;
                marker_positions_.push_back(marker_position_w);

//                cv::namedWindow("aruco test");
//                cv::imshow("aruco test", marker_img);
//                cv::waitKey(1);
                string str_frame_id = to_string(camlidar_process_->keyframeDB()->latest_keyframe()->id());
                string f_name = str_frame_id + "_with_marker.png";
                cv::imwrite(f_name, marker_img);


                cv::Mat marker_pose = cv::Mat::zeros(3, 4, CV_64FC1);

                marker_pose.at <double> (0, 0) = Rot.at<double> (0, 0);   marker_pose.at <double> (0, 1) = Rot.at<double> (0, 1);  marker_pose.at <double> (0, 2) = Rot.at<double> (0, 2);
                marker_pose.at <double> (1, 0) = Rot.at<double> (1, 0);   marker_pose.at <double> (1, 1) = Rot.at<double> (1, 1);  marker_pose.at <double> (1, 2) = Rot.at<double> (1, 2);
                marker_pose.at <double> (2, 0) = Rot.at<double> (2, 0);   marker_pose.at <double> (2, 1) = Rot.at<double> (2, 1);  marker_pose.at <double> (2, 2) = Rot.at<double> (2, 2);

                marker_pose.at <double> (0, 3) = tvecs[0](0);
                marker_pose.at <double> (1, 3) = tvecs[0](1);
                marker_pose.at <double> (2, 3) = tvecs[0](2);

                marker_pose_.push_back(marker_pose);

                marker_frame_ids_.push_back(camlidar_process_->keyframeDB()->latest_keyframe()->id());
                marker_ids_.push_back(ids[0]);
            }
        }
    }

}

void MainWindow::save_marker_positions()
{
    cerr << "Saving marker positions" << endl;

    QFile file("marker_positions");
//    if (file.open(QFile::WriteOnly | QFile::Truncate)) {
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream out(&file);

        for(auto iter=marker_positions_.begin(); iter != marker_positions_.end(); ++iter) {
            out << iter->x() << " " << iter->y() << " " << iter->z() << endl;
        }
        // writes "Result: 3.14      2.7       "
    }

    file.close();
}

void MainWindow::save_marker_pose()
{
    cerr << "Saving marker poses" << endl;

    QFile file("marker_poses");

    if (file.open(QIODevice::ReadWrite)) {
        QTextStream out(&file);

        for(size_t i = 0; i < marker_frame_ids_.size(); ++i) {
            out << marker_frame_ids_[i] << " " << marker_ids_[i] << " "
                << marker_pose_[i].at <double> (0, 0) << " " << marker_pose_[i].at <double> (0, 1) << " " << marker_pose_[i].at <double> (0, 2) << " " << marker_pose_[i].at <double> (0, 3) << " "
                << marker_pose_[i].at <double> (1, 0) << " " << marker_pose_[i].at <double> (1, 1) << " " << marker_pose_[i].at <double> (1, 2) << " " << marker_pose_[i].at <double> (1, 3) << " "
                << marker_pose_[i].at <double> (2, 0) << " " << marker_pose_[i].at <double> (2, 1) << " " << marker_pose_[i].at <double> (2, 2) << " " << marker_pose_[i].at <double> (2, 3) << " "
                << endl;
        }

    }
}

void MainWindow::save_poses()
{
    cerr << "Saving poses of keyframes" << endl;

    QFile file("result_poses");

    if (file.open(QIODevice::ReadWrite)) {
        QTextStream out(&file);

        for(auto iter=frameDB_->begin(); iter != frameDB_->end(); ++iter) {

            for (int i=0; i<4;i++)
            {
                for (int j=0; j<4; j++)
                    out << (*iter)->Twc().matrix()(i,j) << " ";

                out << endl;
            }

//            Twc_list_.push_back(qmat);

        }
    }

    file.close();
}

void MainWindow::save_point_map()
{
    cerr << "Saving point map!" << endl;

    QFile file("result_points");

    if (file.open(QIODevice::ReadWrite)) {
        QTextStream out(&file);

        for(auto iter=keyframeDB_->begin(); iter != keyframeDB_->end(); ++iter) {

            dedvo::PointCloud pc_l = (*iter)->pointcloud();
            dedvo::PointCloud pc_w;
            pcl::transformPointCloud((*iter)->pointcloud(), pc_w, (*iter)->frame()->Twc().matrix());

            for (int i=0; i<pc_w.size(); ++i) {
                if(pc_l[i].y > -1.5 && pc_l[i].y < 0.3 && pc_l[i].z < 20.0) {
                    out << pc_w[i].x << " " << pc_w[i].y << " " << pc_w[i].z << endl;
                }
            }

        }
    }

    file.close();
}
