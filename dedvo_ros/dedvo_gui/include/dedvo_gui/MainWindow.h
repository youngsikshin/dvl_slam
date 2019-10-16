#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QMutex>
#include <QVector>

#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>
#include <dedvo_gui/CamLidarProcess.h>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char* argv[], QWidget *parent = 0);
    ~MainWindow();

    QMutex mutex;

protected:
    void keyPressEvent(QKeyEvent *event);

private slots:
    void set_KF();
    void save_marker_positions();
    void save_marker_pose();
    void save_poses();
    void save_point_map();

    void drawGL();

private:
    Ui::MainWindow *ui;

    std::shared_ptr<ros::NodeHandle> node_;
    std::shared_ptr<ros::NodeHandle> privante_nh_;
    std::thread *camlidar_thread_;
    CamLidarProcess *camlidar_process_;

    dedvo::KeyframeDB::Ptr keyframeDB_;
    dedvo::FrameDB::Ptr frameDB_;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);     //DICT_6X6_1000         DICT_ARUCO_ORIGINAL
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    QVector<QVector3D> marker_positions_;

    QVector<int> marker_frame_ids_;
    QVector<int> marker_ids_;
    QVector<cv::Mat> marker_pose_;

    void detect_aruco(QMatrix4x4 current_pose);

};

#endif // MAINWINDOW_H
