#ifndef DEDVOGLWIDGET_H
#define DEDVOGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLShader>
#include <QMouseEvent>

#include <QMutex>
#include <QtMath>
#include <QVector>
#include <QPoint>
#include <QMatrix4x4>
#include <QTimer>

#include <dedvo/KeyframeDB.h>

class DedvoGLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit DedvoGLWidget(QWidget *parent = 0);
    ~DedvoGLWidget();

    QTimer *timer_;

    bool show_traj;
    float axes_scale;
    QVector <QVector3D> axes_;
    QVector <QVector3D> no_axes_;
    QVector <QVector3D> axes_color_;

    QVector <QVector3D> grid_;
    QVector <QVector3D> red_cube_;
    QVector <QVector3D> gray_cube_;

    QMatrix4x4 init_pose;

    QMutex& Twc_mutex() { return Twc_mutex_; }

    void keyframeDB(dedvo::KeyframeDB::Ptr keyframeDB) { keyframeDB_ = keyframeDB; }
    void frameDB(dedvo::FrameDB::Ptr frameDB) { frameDB_ = frameDB; }
    void Twc_list();
    void pointcloud_list();
    QMatrix4x4& current_pose() { return *(Twc_list_.end()-1); }
    bool is_drawing() { return Twc_list_.size() == pointcloud_list_.size(); }

    void save_framebuffer();

signals:
    void mouseEvent();

public slots:

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void show_camera_info();

    void drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, Qt::GlobalColor color);
    void drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QColor& color);
    void drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QVector <GLfloat> data_color, int stride);
    void drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QVector<QVector3D> data_color);
    void drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QVector<QVector3D> data_color, int stride);
    void draw_background(QOpenGLShaderProgram &shader, QColor& top_color, QColor& bot_color);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

private:
    QPoint last_mouse_position_;

    // for lookat
    QVector3D camera_position_;
    QVector3D camera_front_;
    QVector3D camera_up_direction_;

    bool is_following_;

    QOpenGLShaderProgram shader_program_;
    QOpenGLShaderProgram shader_program_color_;
    QOpenGLShaderProgram bg_shader_;

    // camera motion
    QMatrix4x4 mat_p_;
    QMatrix4x4 mat_m_;
    QMatrix4x4 mat_v_;

    // for mouse motion
    float trans_y_;
    float trans_x_;
    float rot_right_;
    float rot_z;

    QMutex Twc_mutex_;
    QMutex pc_mutex_;

    dedvo::KeyframeDB::Ptr keyframeDB_;
    dedvo::FrameDB::Ptr frameDB_;
    QVector <QMatrix4x4> Twc_list_;
    QVector <QVector<QVector3D>> pointcloud_list_;
    QVector <QVector<QVector3D>> color_list_;   

    int num_framebuffer_;
};

#endif // DEDVOGLWIDGET_H
