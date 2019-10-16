#include <dedvo_gui/DedvoGLWidget.h>

DedvoGLWidget::DedvoGLWidget (QWidget *parent)
    : QOpenGLWidget(parent)
{
//    camera_position_ = QVector3D (0, -1.0, -5);
//    camera_front_ = QVector3D (0, 0, 1);
//    camera_up_direction_ = QVector3D(0, -1, 0);

    camera_position_ = QVector3D (0, -2.5, -15);
    camera_front_ = QVector3D (0, 0, 1);
    camera_up_direction_ = QVector3D(0, -1, 0);

    setFocusPolicy(Qt::StrongFocus);

    is_following_ = true;
    show_traj = false;

    init_pose.setToIdentity();
    init_pose.translate(0.0, 0.0, 1.0);
    init_pose.rotate(-90.0, 1, 0, 0);

    num_framebuffer_ = 0;

    axes_scale = 0.8; // 0.9 0.45

    timer_ = new QTimer(this);
}

DedvoGLWidget::~DedvoGLWidget()
{

}

QSize DedvoGLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize DedvoGLWidget::sizeHint() const
{
    return QSize(400, 400);
}

void DedvoGLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glClearColor(1.0, 1.0, 1.0, 1.0);
//    glClearColor(1.0, 1.0, 0.8, 1.0);
//    glClearColor(0.1, 0.1, 0.1, 1.0);


    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/vertexShader.vsh");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/fragmentShader.fsh");
    shader_program_.link();

    shader_program_color_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/vertexShaderColor.vsh");
    shader_program_color_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/fragmentShaderColor.fsh");
    shader_program_color_.link();

    bg_shader_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/background.vsh");
    bg_shader_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/background.fsh");
    bg_shader_.link();

    axes_ << QVector3D(0, 0, 0) << QVector3D(1, 0, 0)*axes_scale
         << QVector3D(0, 0, 0) << QVector3D(0, 1, 0)*axes_scale
         << QVector3D(0, 0, 0) << QVector3D(0, 0, 1)*axes_scale;

    no_axes_ << QVector3D(0, 0, 0) << QVector3D(1, 0, 0)*0.0
             << QVector3D(0, 0, 0) << QVector3D(0, 1, 0)*0.0
             << QVector3D(0, 0, 0) << QVector3D(0, 0, 1)*0.0;

//    axes_color_ << QVector3D(1, 0, 0) << QVector3D(1, 0, 0)
//               << QVector3D(0, 1, 0) << QVector3D(0, 1, 0)
//               << QVector3D(0, 0, 1) << QVector3D(0, 0, 1);
    axes_color_ << QVector3D(0, 1, 0) << QVector3D(0, 1, 0)
               << QVector3D(0, 0, 1) << QVector3D(0, 0, 1)
               << QVector3D(1, 0, 0) << QVector3D(1, 0, 0);


    int max_size = 10;
    for (int i=0; i<=max_size; i++) {
//        grid_.push_back(QVector3D(-max_size, i, 0)); grid_.push_back(QVector3D(max_size, i, 0));
//        grid_.push_back(QVector3D(-max_size, -i, 0)); grid_.push_back(QVector3D(max_size, -i, 0));
//        grid_.push_back(QVector3D(i, -max_size, 0)); grid_.push_back(QVector3D(i, max_size, 0));
//        grid_.push_back(QVector3D(-i, -max_size, 0)); grid_.push_back(QVector3D(-i, max_size, 0));
          grid_.push_back(QVector3D( i, 1, -max_size)); grid_.push_back(QVector3D(i, 1, max_size));
          grid_.push_back(QVector3D(-i, 1, -max_size)); grid_.push_back(QVector3D(-i, 1, max_size));
          grid_.push_back(QVector3D(-max_size, 1, i)); grid_.push_back(QVector3D(max_size, 1, i));
          grid_.push_back(QVector3D(-max_size, 1, -i)); grid_.push_back(QVector3D(max_size, 1, -i));
    }

    float a = 0.15;
    float b = 0.15;
    float c = 0.15;
    red_cube_ << QVector3D(a, -b, c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c)
              << QVector3D(a, b, -c) << QVector3D(a, b, c) << QVector3D(a, -b, c) // 1
              << QVector3D(-a, -b, c) << QVector3D(-a, -b, -c) << QVector3D(a, -b, -c)
              << QVector3D(a, -b, -c) << QVector3D(a, -b, c) << QVector3D(-a, -b, c) // 2
              << QVector3D(a, b, c) << QVector3D(a, b, -c) << QVector3D(-a, b, -c)
              << QVector3D(-a, b, -c) << QVector3D(-a, b, c) << QVector3D(a, b, c) //3
              << QVector3D(-a, -b, c) << QVector3D(a, -b, c) << QVector3D(a, b, c)
              << QVector3D(a, b, c) << QVector3D(-a, b, c) << QVector3D(-a, -b, c) // 4
              << QVector3D(-a, -b, -c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c)
              << QVector3D(a, b, -c) << QVector3D(-a, b, -c) << QVector3D(-a, -b, -c) // 5
              << QVector3D(-a, -b, c) << QVector3D(-a, -b, -c) << QVector3D(-a, b, -c)
              << QVector3D(-a, b, -c) << QVector3D(-a, b, c) << QVector3D(-a, -b, c) //6;
              << QVector3D(a, -b, c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c);


}

void DedvoGLWidget::paintGL()
{
//    show_camera_info();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    QColor top_color(Qt::black);
//    QColor bot_color(Qt::lightGray);
    QColor top_color(Qt::white);
    QColor bot_color(Qt::white);
    draw_background(bg_shader_, top_color, bot_color);

    mat_m_.setToIdentity();
    mat_v_.setToIdentity();

    mat_v_.lookAt(camera_position_, camera_position_+camera_front_, camera_up_direction_);

//    mat_m_.rotate(90.0, 1, 0, 0);
    QMatrix4x4 T = mat_p_ * mat_v_ * mat_m_;

//    glLineWidth(1);
//    drawShader(shader_program_, GL_LINES, T, grid_, Qt::gray);

//    mat_m_.setToIdentity();
//    T = mat_p_ * mat_v_ * mat_m_;

//    if(show_traj) {
        glLineWidth(3);
//        drawShader(shader_program_color_, GL_LINES, T, axes_, axes_color_);
//    }

    pc_mutex_.lock();

    auto pc_iter = pointcloud_list_.begin();
    auto color_iter = color_list_.begin();
    glPointSize(1.0);

    auto latest_T = Twc_list_.end()-1;

    for(auto iter = Twc_list_.begin(); iter != Twc_list_.end(); ++iter, ++pc_iter, ++color_iter) {

//        if ( (latest_T->column(3).toVector3D() - iter->column(3).toVector3D()).length() > 6.0)
//            continue;

        T = mat_p_ * mat_v_ * (*iter);

        if(show_traj) {
           drawShader(shader_program_color_, GL_LINES, T, axes_, axes_color_);
        } else {
           drawShader(shader_program_color_, GL_LINES, T, no_axes_, axes_color_);
        }

        if(pc_iter == pointcloud_list_.end()) break;
        if(color_iter == color_list_.end()) break;

        drawShader(shader_program_color_, GL_POINTS, T, (*pc_iter), (*color_iter), 1);
    }
    pc_mutex_.unlock();

    int num_cube = 10;
    int interval = 8;
    auto end_iter = latest_T - num_cube * interval;
    QColor cube_color;
    int cnt_cube = 0;

    for(auto iter = latest_T; iter != end_iter; iter = iter - interval) {

        if(iter == latest_T) {
            cube_color = QColor(Qt::red);
            ++cnt_cube;
            continue;
        }
        else {
            float color_factor = static_cast<float>(num_cube-cnt_cube)/static_cast<float>(num_cube);
            int i_color_factor = static_cast<int> (color_factor*255);
            cube_color = QColor(i_color_factor, i_color_factor, i_color_factor);
        }

        cube_color.setAlphaF(static_cast<float>(num_cube-cnt_cube)/static_cast<float>(num_cube));
        cube_color.setAlphaF(0.0);

        QMatrix4x4 cube_T = mat_p_ * mat_v_ * (*iter);
        QVector<QVector3D> red_point;
        red_point.push_back(QVector3D(0.0, 0.0, 0.0));
        glPointSize(5.0);


//        drawShader(shader_program_, GL_TRIANGLE_STRIP, cube_T, red_cube_, cube_color);
        drawShader(shader_program_, GL_POINTS, cube_T, red_point, cube_color);

        ++cnt_cube;
        if(Twc_list_.size() < cnt_cube*interval) break;

    }

    glLineWidth(3.5);
    T = mat_p_ * mat_v_ * (*latest_T);
    drawShader(shader_program_color_, GL_LINES, T, axes_, axes_color_);
}

void DedvoGLWidget::resizeGL(int width, int height)
{
    if (height ==0) {
        height = 1;
    }

    mat_p_.setToIdentity();
    mat_p_.perspective(60.0, (float) width/float(height), 0.001, 1000);

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

}

void DedvoGLWidget::draw_background(QOpenGLShaderProgram &shader, QColor& top_color, QColor& bot_color)
{
    shader.bind();

    glDisable(GL_DEPTH_TEST);
    shader.setUniformValue("top_color", top_color);
    shader.setUniformValue("bot_color", bot_color);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);

    shader.release();
}


void DedvoGLWidget::drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, Qt::GlobalColor color)
{
    shader.bind();
    shader.setUniformValue("color", QColor(color));
    shader.setAttributeArray("vertex", data.constData());
    shader.setUniformValue("mvpMatrix", T);
    shader.enableAttributeArray("vertex");
    glDrawArrays(type, 0, data.size());
    shader.disableAttributeArray("vertex");
    shader.release();
}

void DedvoGLWidget::drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QColor& color)
{
    shader.bind();
    shader.setUniformValue("color", color);
    shader.setAttributeArray("vertex", data.constData());
    shader.setUniformValue("mvpMatrix", T);
    shader.enableAttributeArray("vertex");
    glDrawArrays(type, 0, data.size());
    shader.disableAttributeArray("vertex");
    shader.release();
}

void DedvoGLWidget::drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QVector <GLfloat> data_color, int stride)
{
    shader.bind();
    shader.setAttributeArray("vertex", data.constData(), 3, 3*stride*sizeof(GLfloat));
    shader.enableAttributeArray("vertex");
    shader.setAttributeArray("color", data_color.constData(), 3, 3*stride*sizeof(GLfloat));
    shader.enableAttributeArray("color");
    shader.setUniformValue("mvpMatrix", T);
    if (stride==0) stride = 1;
    glDrawArrays(type, 0, data.size()/(3*stride));
    shader.disableAttributeArray("color");
    shader.disableAttributeArray("vertex");
    shader.release();
}

void DedvoGLWidget::drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QVector<QVector3D> data_color)
{
    shader.bind();
    shader.setAttributeArray("vertex", data.constData(), sizeof(QVector3D));
    shader.enableAttributeArray("vertex");
    shader.setAttributeArray("color", data_color.constData(), sizeof(QVector3D));
    shader.enableAttributeArray("color");
    shader.setUniformValue("mvpMatrix", T);
    glDrawArrays(type, 0, data.size());
    shader.disableAttributeArray("color");
    shader.disableAttributeArray("vertex");
    shader.release();
}

void DedvoGLWidget::drawShader(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QVector<QVector3D> data_color, int stride)
{
  shader.bind();
  shader.setAttributeArray("vertex", data.constData(), sizeof(QVector3D)*stride);
  shader.enableAttributeArray("vertex");
  shader.setAttributeArray("color", data_color.constData(), sizeof(QVector3D)*stride);
  shader.enableAttributeArray("color");
  glDrawArrays(type, 0, data.size()/stride);
  shader.disableAttributeArray("color");
  shader.disableAttributeArray("vertex");
  shader.setUniformValue("mvpMatrix", T);
  shader.release();
}

void DedvoGLWidget::mousePressEvent(QMouseEvent *event)
{
    last_mouse_position_ = event->pos();
    event->accept();
}

void DedvoGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    float delta_x = event->x() - last_mouse_position_.x();
    float delta_y = last_mouse_position_.y() - event->y();
    float sensitivity = 0.25;

    if (event->buttons() & Qt::LeftButton) { // translation motion
        QVector3D left = QVector3D::normal(camera_up_direction_, camera_front_);
        camera_position_ = camera_position_ - (camera_position_.z()*0.01*sensitivity)*delta_y*camera_up_direction_;
        camera_position_ = camera_position_ + (camera_position_.z()*0.01*sensitivity)*delta_x*left;
        trans_y_ = delta_y;
        trans_x_ = delta_x;
    }

    if (event->buttons() & Qt::RightButton) { // rotation motion
        delta_x *= sensitivity;
        delta_y *= sensitivity;


        QMatrix4x4 rotation_right;
        QMatrix4x4 rotation_z;
        QMatrix4x4 rotation;

        rotation.setToIdentity();
        QVector3D right = QVector3D::normal(camera_front_, camera_up_direction_);

        rotation.rotate(delta_y, right);
//        rotation.rotate(-delta_x, 0, 0, 1);
        rotation.rotate(-delta_x, 0, -1, 0);


        rotation_right.rotate(delta_y, right);
//        rotation_z.rotate(-delta_x, 0, 0, 1);
        rotation.rotate(-delta_x, 0, -1, 0);

        QVector3D temp_pose;
        QVector3D init_pose(0, 0, 0);

        if (qFabs(delta_x) > qFabs(delta_y)) {

        }
        else {

        }

        temp_pose = rotation_right*camera_position_;
        camera_position_ = rotation_z*init_pose + temp_pose;
//        _camPosition = rotation*_camPosition;

        camera_front_ = rotation*camera_front_;
        camera_up_direction_ = rotation*camera_up_direction_;
    }

    last_mouse_position_ = event->pos();
    event->accept();

//    update();
//    save_framebuffer();
}

void DedvoGLWidget::wheelEvent(QWheelEvent *event)
{
    int delta = (float)event->delta();
    if (event->orientation() == Qt::Vertical) {
        if (delta < 0) {
            camera_position_ -= 0.8 * camera_front_;
        } else if (delta > 0) {
            camera_position_ += 0.8 * camera_front_;
        }

//        update();
//        save_framebuffer();
    }
}

void DedvoGLWidget::keyPressEvent(QKeyEvent *event)
{
    float camera_speed = 0.05f;
    if (event->key() == Qt::Key_W) {
        camera_position_ += camera_speed * camera_front_;
    }
    if (event->key() == Qt::Key_S) {
        camera_position_ -= camera_speed * camera_front_;
    }
    if (event->key() == Qt::Key_A) {
        camera_position_ -= QVector3D::crossProduct(camera_front_,camera_up_direction_).normalized() * camera_speed;
    }
    if (event->key() == Qt::Key_D) {
        camera_position_ += QVector3D::crossProduct(camera_front_,camera_up_direction_).normalized() * camera_speed;
    }
    if (event->key() == Qt::Key_F) {
        is_following_ = is_following_ ? false : true;
    }
    if (event->key() == Qt::Key_T) {
        show_traj = show_traj ? false : true;
    }
    if (event->key() == Qt::Key_Y) {
//        camera_position_ = QVector3D (-12, -10, 4);
//        camera_front_ = QVector3D (0, 1, 0);
//        camera_up_direction_ = QVector3D(0, 0, 1);

      //samsung
//      camera_position_ = QVector3D (36.9391, -51.5306, -11.3653);
//      camera_front_ = QVector3D (0.00107667, 0.99405, 0.108841);
//      camera_up_direction_ = QVector3D(0.0178002, 0.108805, -0.993881);

      camera_position_ = QVector3D (36.8337, -52.4276, -6.60963);
      camera_front_ = QVector3D (0.00232005, 0.999836, 0.0174132);
      camera_up_direction_ = QVector3D(0.000, 0.00, -1.00);

    }
    if (event->key() == Qt::Key_P) {
      save_framebuffer();
    }

//    update();
//    save_framebuffer();
}

void DedvoGLWidget::Twc_list()
{
    Twc_mutex_.lock();
    Twc_list_.clear();

////    auto iter = keyframeDB_->end()-1;
//    for(auto iter=keyframeDB_->begin(); iter != keyframeDB_->end(); ++iter) {
//  //        cerr << (*iter)->frame()->Twc().matrix() << endl;

//  //        Eigen::Matrix<qreal, 4, 4, Eigen::RowMajor> mcopy = (*iter)->frame()->Twc().matrix().cast<qreal>();
//  //        QMatrix4x4 qmat(mcopy.data());
//        QMatrix4x4 qmat;
//        for (int i=0; i<4;i++)
//        {
//            for (int j=0; j<4; j++)
//                qmat(i,j)=(*iter)->frame()->Twc().matrix()(i,j);
//        }

//        Twc_list_.push_back(qmat);

//    }

    vector<dedvo::Frame::Ptr>& frameDB = frameDB_->frameDB();
    for(dedvo::Frame::Ptr frame_ptr:frameDB) {
        QMatrix4x4 qmat;
        for (int i=0; i<4; ++i) {
            for (int j=0; j<4; ++j) {
                qmat(i,j) = frame_ptr->Twc().matrix()(i,j);
            }
        }

        Twc_list_.push_back(qmat);
    }

    Twc_mutex_.unlock();

    auto cur_pose = Twc_list_.back();

    if(Twc_list_.size() > 0 && is_following_) {

        float back_dist_offset = 2.0;  // kitti 30.0 indoor 2.0 3.2 15.0
        float up_dist_offset = 0.8;     // kitti 2.5 indoor 0.8 1.2 4.0

        camera_position_.setX( (cur_pose)(0,3) - back_dist_offset*(cur_pose)(0,2) - up_dist_offset*((cur_pose)(0,1)) );   //  - up_dist_scale*((*cur_pose)(0,1))
        camera_position_.setY( (cur_pose)(1,3) - back_dist_offset*(cur_pose)(1,2) - up_dist_offset*((cur_pose)(1,1)) );
        camera_position_.setZ( (cur_pose)(2,3) - back_dist_offset*(cur_pose)(2,2) - up_dist_offset*((cur_pose)(2,1)) );   // - up_dist_scale*((*cur_pose)(2,1)) *(*cur_pose)(2,2)

        camera_front_.setX((cur_pose)(0,2));
        camera_front_.setY((cur_pose)(1,2));
        camera_front_.setZ((cur_pose)(2,2));

        camera_up_direction_.setX(-((cur_pose)(0,1))-((cur_pose)(0,2)));
        camera_up_direction_.setY(-((cur_pose)(1,1))-((cur_pose)(1,2)));
        camera_up_direction_.setZ(-((cur_pose)(2,1))-((cur_pose)(2,2)));

        // Samsung
//            camera_position_ = QVector3D (24.8038, -27.3624, -0.325261);
//            camera_front_ = QVector3D (-0.540393, 0.700368, 0.46637);
//            camera_up_direction_ = QVector3D(-0.444835, -0.708255, 0.548207);

//            camera_position_ = QVector3D (16.2437, -48.2954, 27.9037);
//            camera_front_ = QVector3D (-0.283254, 0.955856, 0.0786032);
//            camera_up_direction_ = QVector3D(-0.800608, -0.280739, 0.529478);

//        camera_position_ = QVector3D (11.3455, -51.5456, 35.4298);
//        camera_front_ = QVector3D (-0.181802, 0.983243, -0.0139856);
//        camera_up_direction_ = QVector3D(-0.982221, -0.182258, -0.0453254);


    }

}

void DedvoGLWidget::pointcloud_list()
{
//    pointcloud_list_.clear();
//    color_list_.clear();

//    for (auto iter=keyframeDB_->begin(); iter != keyframeDB_->end(); ++iter) {

//    auto iter = keyframeDB_->end()-1;
//        QVector<QVector3D> pointcloud;
//        QVector<QVector3D> color;

//        for(auto pc_iter = (*iter)->frame()->pointcloud().begin(); pc_iter != (*iter)->frame()->pointcloud().end(); ++pc_iter) {
////        for(auto pc_iter = (*iter)->pointcloud().begin(); pc_iter != (*iter)->pointcloud().end(); ++pc_iter) {
////            QVector3D points( pc_iter->point(0), pc_iter->point(1), pc_iter->point(2) );

//            if (pc_iter->a == 0) continue;

//            QVector3D points( pc_iter->x, pc_iter->y, pc_iter->z );
//            QVector3D colors( static_cast<float> (pc_iter->r) / 255.0, static_cast<float> (pc_iter->g) / 255.0, static_cast<float> (pc_iter->b) / 255.0 );
////            if(pc_iter->y > -1.5 && pc_iter->y < 0.3 && pc_iter->z < 5.0) {
//            if(pc_iter->a != 0) { // samsung x->1
//                pointcloud.push_back(points);
//                color.push_back(colors);
//            }
//        }

//        cerr << "DB size is " << frameDB_->size() << endl;
//        if(frameDB_->size() < 1) return;

        auto iter = frameDB_->end()-1;

        QVector<QVector3D> pointcloud;
        QVector<QVector3D> color;

        int cnt=0;
        for(auto pc:(*iter)->pointcloud()) {
            if(pc.a == 0) continue;
            if(cnt++%10 != 0) continue;
//            if(pc.y < -0.8 || pc.z > 8.0) continue;

            QVector3D points( pc.x, pc.y, pc.z);
            QVector3D colors( static_cast<float> (pc.r)/ 255.0, static_cast<float> (pc.g)/255.0, static_cast<float> (pc.b)/255.0);

            if(pc.a != 0) {
                pointcloud.push_back(points);
                color.push_back(colors);
            }
        }

        pc_mutex_.lock();

        pointcloud_list_.push_back(pointcloud);
        color_list_.push_back(color);

        pc_mutex_.unlock();
//    }


}

void DedvoGLWidget::save_framebuffer()
{

    QImage screen_shot = grabFramebuffer();
    bool result = screen_shot.save("./" + QString::number(num_framebuffer_) + ".png");
    num_framebuffer_++;

}

void DedvoGLWidget::show_camera_info()
{
    cerr << "camera_position_ = QVector3D (" << camera_position_.x() << ", " << camera_position_.y() << ", " << camera_position_.z() << ");" << endl;
    cerr << "camera_front_ = QVector3D (" << camera_front_.x() << ", " << camera_front_.y() << ", " << camera_front_.z() << ");" << endl;
    cerr << "camera_up_direction_ = QVector3D(" << camera_up_direction_.x() << ", " << camera_up_direction_.y() << ", " << camera_up_direction_.z() << ");" << endl;
}
