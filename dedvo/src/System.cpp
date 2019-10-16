#include <dedvo/System.h>
#include <dedvo/Conversion.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/registration/gicp.h>

namespace dedvo {

System::System()
    :initialize_(false)
{
    cerr << "[System]\t Called constructor of system" << endl;
}

System::System(string path, string fname)
    : initialize_(false)
{
    cerr << "[System]\t Called constructor with a configuration file" << endl;

    Config::cfg(path, fname);

    auto camera_info = Config::cfg()->camera_info();
    auto voca_fname = Config::cfg()->loopclosure().BoW_fname;

    extrinsics_ = Config::cfg()->camlidar().extrinsic;

    camera_ = Config::cfg()->camera();

    tracker_.reset(new dedvo::Tracker());
    tracker2_.reset(new Tracker2());

    vocabulary_.reset( new ORBVocabulary(voca_fname) );

    keyframeDB_.reset(new KeyframeDB(vocabulary_));

    frameDB_.reset(new FrameDB());

    loop_closing_.reset(new LoopClosing(keyframeDB_, vocabulary_, camera_));

    num_keyframe_ = 3;

    kf_window_.reset(new KeyframeWindow(num_keyframe_));
    window_optimizer_.reset((new WindowOptimizer2(kf_window_)));
}

System::~System()
{
    // extrinsics_ <<
    cerr << "[System]\t Called destructor of system" << endl;
    loop_closing_->request_finish();

     if (keyframeDB_->size() > num_keyframe_-1) {
         vector<Keyframe::Ptr> keyframe_window;
         keyframeDB_->latest_keyframe(keyframe_window, num_keyframe_-1);

         for(int i=0; i<num_keyframe_-2; i++) {
             cout << keyframe_window[i+1]->frame()->Twc().matrix() << endl;
         }
     }

    cerr << "[System]\t End destructor of system" << endl;
}

bool System::track_camlidar(Frame::Ptr current)
{   
    if(!initialize_) {

        Eigen::Matrix3f rot;
        rot << 1.0,  0.0, 0.0,
               0.0,  1.0, 0.0,
               0.0,  0.0, 1.0;

//      indoor
//        rot << 0.994521895368273,                 0.0,   0.104528463267653,
//              -0.010926199633097,   0.994521895368273,   0.103955845408880,
//              -0.103955845408880,  -0.104528463267653,   0.989073800366903;

//      outdoor
//        rot <<   0.714182546347143,  -0.087155742747658,  -0.694512179158304,
//                 0.699479763400873,   0.052136802128782,   0.712748072222186,
//                -0.025910443565680,  -0.994829447880333,  0.098198872418890;

        Eigen::Vector3f twc(0, 0, 0); // for kitti
        Sophus::SE3f sophus_twc(rot, twc);

        cout<< sophus_twc.matrix() << endl;

        Keyframe::Ptr current_keyframe(new Keyframe(current, id_manager.id(), camera_, vocabulary_));

        current_keyframe->ORB_extractor();
        current_keyframe->compute_BoW();
        current_keyframe->frame()->Twc(sophus_twc);
        keyframeDB_->add(current_keyframe);
        initialize_ = true;

        Sophus::SE3d pose = current_keyframe->frame()->dTwc();
        graph_optimizer_.add_pose_to_graph(pose, current_keyframe->id());

        frameDB_->add(current);


        // for seq 12
//        Eigen::Vector3f dtji(0, 0, -1.5); // for kitti
//        Sophus::SE3f sophus_dtji(rot, dtji);
//        dTji_ = sophus_dtji;

        return true;
    
    }

    Keyframe::Ptr latest_keyframe = keyframeDB_->latest_keyframe();

//    latest_keyframe->frame()->show_image_with_points(latest_keyframe->frame()->level(0),0);

    Sophus::SE3f prev_Tji = Tji_;
    Tji_ = Tji_ * dTji_;

//    auto tic = timestamp_now();
//    tracking_status_ = tracker_->match(latest_keyframe, current, Tji_);

//    if(dTji_.log().block<3,1>(3,0).norm()<0.01) tracker2_->max_level(1);
//    else if (dTji_.log().block<3,1>(0,0).norm()<0.01) tracker2_->max_level(3);
//    else tracker2_->max_level(2);

//    if(latest_keyframe->id() > 50) tracker2_->max_level(0); // max level 0 for highway data

    tracking_status_ = tracker2_->tracking(latest_keyframe, current, Tji_);
//    cerr << endl << Tji_.matrix() << endl;
//    auto toc = timestamp_now();

//    cerr << "[System]\t Computation time and rate : " << (toc-tic)*1e-6 << ", " << 1/((toc-tic)*1e-6)<< endl;

    dTji_ = Tji_ * prev_Tji.inverse();

    Tij_ = Tji_.inverse();
    current->Twc(latest_keyframe->frame()->Twc()*Tij_);
//    current->Twc(latest_keyframe->frame()->Twc()*prev_Tji.inverse());

    // Keyframe decision
//    bool is_keyframe = false;

    auto v_rot_ji = dTji_.log().block<3,1>(3,0).norm();
    auto v_t_ji = dTji_.log().block<3,1>(0,0).norm();

    cerr << "norm: " <<  v_t_ji << " , " << v_rot_ji << endl;

    float ratio_threshold = 1.0;
//    if ((v_rot_ji < 0.005 || v_t_ji < 0.5) && latest_keyframe->id() > 50) { // rotational
//        ratio_threshold = 0.85;
//    }
//    if (v_rot_ji > 0.01)  ratio_threshold = 1.0;
//    if (v_t_ji > 1.50) ratio_threshold = 1.0;

//    current->show_image_with_points(current->original_img(), 0);

    Keyframe::Ptr current_keyframe(new Keyframe(current, camera_, vocabulary_));

//    current_keyframe->show_image_with_points(current_keyframe->frame()->original_img(), 0);

    float visible_ratio1 = latest_keyframe->get_visible_ratio(current_keyframe);
    float visible_ratio2 = current_keyframe->get_visible_ratio(latest_keyframe);

    bool is_keyframe = (visible_ratio1 < ratio_threshold ? true : false) || ((visible_ratio2 < ratio_threshold ? true : false));

//    auto visible_points = tracker_->visible_points_in_cur();
//    int num_vis_points = 0;
//    for( auto iter=visible_points.begin(); iter != visible_points.end(); ++iter) {
//        if(*iter) num_vis_points++;
//    }

//    float visible_ratio = static_cast<float> (num_vis_points)/visible_points.size();

//    auto visible_points2 = tracker_->visible_points_in_prev();
//    int num_vis_points2 = 0;
//    for( auto iter=visible_points2.begin(); iter != visible_points2.end(); ++iter) {
//        if(*iter) num_vis_points2++;
//    }

//    float visible_ratio2 = static_cast<float> (num_vis_points2)/visible_points2.size();
////    cerr << "visible ratio : " << visible_ratio << " / " << visible_ratio2 << endl;

//    is_keyframe = visible_ratio < 1.8 ? true : false;

//    bool is_keyframe2 = visible_ratio2 < 1.8 ? true : false;

    if(is_keyframe) {
//        Keyframe::Ptr current_keyframe(new Keyframe(current, id_manager.id(), camera_, vocabulary_));
        current_keyframe->id(id_manager.id());

        current_keyframe->ORB_extractor();
        current_keyframe->compute_BoW();
        current_keyframe->first_connection(false);
        current_keyframe->parent(latest_keyframe);
        latest_keyframe->child(current_keyframe);

        keyframeDB_->add(current_keyframe);
        kf_window_->add(current_keyframe);
//        cerr << "[System]\t Size of Keyframe Window : " << kf_window_->size() << endl;

        Sophus::SE3d pose = current_keyframe->frame()->dTwc();
        graph_optimizer_.add_pose_to_graph(pose, current_keyframe->id());

        Sophus::SE3d edge = Tij_.cast<double>();

        Matrix6x6 info = Matrix6x6::Identity(6, 6);
//        info.block<3,3>(3,3) = info.block<3,3>(3,3)*100;
//        info(0,0) = tracker_->information()(0,0);
//        info(1,1) = tracker_->information()(1,1);
//        info(2,2) = tracker_->information()(2,2);
//        info(3,3) = tracker_->information()(3,3);
//        info(4,4) = tracker_->information()(4,4);
//        info(5,5) = tracker_->information()(5,5);
//        info = info / (tracker_->f_num_measurement());

//            is_prev_info = true;
//            prev_info_ = info;
//        Matrix6x6 info = tracker_->information()/(tracker_->f_num_measurement());

//        info.block<3,3>(0,0) = tracker_->information().block<3,3>(0,0)/(tracker_->f_num_measurement());
//        info.block<3,3>(3,3) = tracker_->information().block<3,3>(3,3)/(tracker_->f_num_measurement());

        graph_optimizer_.add_edge(edge, info, latest_keyframe->id(), current_keyframe->id());
//        graph_optimizer_.add_edge(edge, tracker_->information()/tracker_->f_num_measurement(), latest_keyframe->id(), current_keyframe->id());

//        cerr << tracker_->information() << endl << endl;;
//        cerr << tracker_->f_num_measurement() << endl << endl;
//            cerr << info << endl;

        Sophus::SE3f tmp;
        Tji_ = tmp;

        if(keyframeDB_->size() >= num_keyframe_) {
//            vector<Keyframe::Ptr> keyframe_window;
//            keyframeDB_->latest_keyframe(keyframe_window, num_keyframe_);

//            WindowOptimizer window_optimizer(camera_, kf_window_->frames());
//            window_optimizer.optimize();

//            window_optimizer_->refine();

//            if (v_rot_ji > 0.01 || v_t_ji > 1.0) {
//                vector<Keyframe::Ptr> keyframe_window_lt;
//                keyframeDB_->latest_keyframe(keyframe_window_lt, num_keyframe_);
//                LocalTracker local_tracker(camera_);
//                local_tracker.tracking(kf_window_->frames());
//            }

        }

        if (false) {
//        if (keyframeDB_->size() > num_keyframe_) {
            vector<Keyframe::Ptr> keyframe_window;
            keyframeDB_->latest_keyframe(keyframe_window, num_keyframe_);

//            cerr << "visible ratio: " << keyframe_window[5]->get_visible_ratio(keyframe_window[0]) << endl;
//            keyframeDB_->show_image_with_accum_points(num_keyframe_, 1);

//            WindowOptimizer window_optimizer(camera_, keyframe_window);
//            window_optimizer.optimize();

            auto T_ij_p = keyframe_window[num_keyframe_-3]->frame()->Twc().inverse() * keyframe_window[num_keyframe_-2]->frame()->Twc();
            auto T_ij_c = keyframe_window[num_keyframe_-2]->frame()->Twc().inverse() * keyframe_window[num_keyframe_-1]->frame()->Twc();

            auto v_t_ij_p = T_ij_p.log();
            auto v_t_ij_c = T_ij_c.log();
            bool retracking=false;

            auto v_t_p = v_t_ij_p.block<3,1>(0,0).norm();
            auto v_t_c = v_t_ij_c.block<3,1>(0,0).norm();
            auto v_t_p_r = v_t_ij_p.block<3,1>(3,0).norm();
            auto v_t_c_r = v_t_ij_c.block<3,1>(3,0).norm();

////            cerr << "Retracking? " << v_t_ij_c.norm() / v_t_ij_p.norm() << endl;
////            if ( v_t_ij_c.norm() / v_t_ij_p.norm() > 1.3 || v_t_ij_c.norm() / v_t_ij_p.norm() < 0.7) retracking = true;

            if(v_t_c/v_t_p > 1.5 || v_t_c/v_t_p < 0.5)  retracking = true;
            if(v_t_c_r/v_t_p_r > 1.5 || v_t_c_r/v_t_p_r < 0.5) retracking = true;

            if(false) {
                // gicp
                // downsample clouds
                PointCloud::Ptr modelcloud;
                PointCloud::Ptr modelcloud_downsampled;
                modelcloud_downsampled.reset(new PointCloud());
                PointCloud::Ptr modelcloud_downsampled2;
                modelcloud_downsampled2.reset(new PointCloud());
                PointCloud::Ptr datacloud;
                PointCloud::Ptr datacloud_downsampled;
                datacloud_downsampled.reset(new PointCloud());
                PointCloud::Ptr transformed;
                transformed.reset(new PointCloud());

                PointCloud::Ptr modelcloud_filtered;
                modelcloud_filtered.reset(new PointCloud());

                PointCloud tmp_model;
                auto tmpTj = keyframe_window[num_keyframe_-2]->frame()->Twc();
                tmp_model = keyframe_window[num_keyframe_-2]->frame()->pointcloud();

                for(int i=0; i<num_keyframe_-2; ++i) {
                    auto tmpTi = keyframe_window[i]->frame()->Twc();
                    PointCloud tmp_pc = keyframe_window[i]->frame()->pointcloud();
                    pcl::transformPointCloud(tmp_pc, tmp_pc, (tmpTj.inverse()*tmpTi).matrix());
                    tmp_model += tmp_pc;
                }

                PointCloud tmp_data;
                tmp_data = keyframe_window[num_keyframe_-1]->pointcloud();

                modelcloud = tmp_model.makeShared();
                datacloud = tmp_data.makeShared();

                pcl::VoxelGrid<POINT> vg;
                vg.setInputCloud(modelcloud);
                vg.setLeafSize (0.1f, 0.1f, 0.1f);
                vg.filter(*modelcloud_downsampled);

                pcl::VoxelGridCovariance<POINT> vgc;
                vgc.setInputCloud(modelcloud);
                vgc.setLeafSize(0.6f, 0.6f, 0.6f);
                vgc.setMinimumPointsNumberPerVoxel(50);
                vgc.setMinPointPerVoxel(50);
                vgc.filter(*modelcloud_downsampled2);

//                auto leaves = vgc.getLeaves();
//                for(int i=0; i<leaves.size();++i) {
//                    auto leaf_evals = leaves[i].getEvals();

//                    double p = 2*(leaf_evals(1) - leaf_evals(0))/(leaf_evals(0)+leaf_evals(1)+leaf_evals(2));
//                    if ( p > 0.7 ) {

//                    }
////                    cerr << leaves[i].getPointCount() << endl << endl;)
////                      cerr << leaf_evals << endl << endl;
//                }

                for(auto point:modelcloud_downsampled->points) {

                    auto leaf = vgc.getLeaf(point);

                    auto leaf_evals = leaf->getEvals();
                    double p = (leaf_evals(0))/(leaf_evals(0)+leaf_evals(1)+leaf_evals(2));
                    if ( p > 0.01 ) {
                        modelcloud_filtered->points.push_back(point);
                    }
                }

//                vg.setInputCloud(datacloud);
    //            vg.setLeafSize (0.01f, 0.01f, 0.01f);
    //            vg.filter(*datacloud_downsampled);

                if(modelcloud_filtered->size() > 99) {
                    pcl::GeneralizedIterativeClosestPoint<POINT, POINT> gicp;
                    gicp.setMaxCorrespondenceDistance(0.5);
                    gicp.setInputSource(datacloud);
                    gicp.setInputTarget(modelcloud_filtered);
                    cerr << modelcloud_filtered->size() << endl;

                    auto pc_Ti = keyframe_window[num_keyframe_-2]->frame()->Twc();
                    auto pc_Tj = keyframe_window[num_keyframe_-1]->frame()->Twc();
                    auto pc_Tij = pc_Ti.inverse() * pc_Tj;


                    gicp.align(*transformed);//, pc_Tij.matrix());

                    auto gicp_Tij = gicp.getFinalTransformation();
                    Sophus::SE3f sophus_pc_Tij(gicp_Tij.block<3,3>(0,0), gicp_Tij.block<3,1>(0,3));

                    if (fabs(v_t_ij_c.norm() / v_t_ij_p.norm() - 1.0) > fabs(sophus_pc_Tij.log().norm() / v_t_ij_p.norm() - 1.0) ) {
                        cerr << gicp.getFinalTransformation() << endl << endl;
                        keyframe_window[num_keyframe_-1]->frame()->Twc(keyframe_window[num_keyframe_-2]->frame()->Twc()*sophus_pc_Tij);
                    }

                }
            }






//            cout << keyframe_window[0]->frame()->Twc().matrix() << endl;
            // cerr << Tji_.matrix() << endl;
            // auto Ti = keyframe_window[0]->frame()->Twc();
            // auto Tj = keyframe_window[1]->frame()->Twc();
            // auto Tji = Tj.inverse()*Ti;

            // cerr << Tji.matrix() << endl;

//            unique_lock<mutex> ulock(keyframeDB_->mtx_DB);

//            auto it_end = keyframeDB_->end();
//            for(auto it=keyframeDB_->begin(); it != it_end; ++it) {
//                Eigen::Isometry3f estimate_pose;
//                sophus_se3f_to_eigen_isometry3f((*it)->frame()->Twc(), estimate_pose);
//                graph_optimizer_.add_pose_to_graph(estimate_pose, (*it)->id());
//                cerr << (*it)->id() << endl;
//            }

//            for(auto it=keyframeDB_->begin(); it != it_end-1; ++it) {

//                Sophus::SE3f Tij = (*it)->frame()->Twc().inverse()*(*(it+1))->frame()->Twc();
//                Eigen::Isometry3f measured_Tij;
//                sophus_se3f_to_eigen_isometry3f(Tij, measured_Tij);

//                cerr << (*(it))->id() << ", " << (*(it+1))->id() << " / " << keyframeDB_->size() << endl;
//                graph_optimizer_.add_edge(measured_Tij, (*(it))->id(), (*(it+1))->id());

//            }

        } else {

        }

//        Keyframe::compare_triangle(current_keyframe, latest_keyframe);

        // Loop Closure
        loop_closing_->insert_keyframe(current_keyframe);
        if (loop_closing_->run_without_thread()){
            cerr << " [System]\t loop detected!! " << endl;
            if( loop_closing_->loop_constraint() ) {

                // gicp
//                Sophus::SE3d edge;
//                if(true) {
//                    auto current_id = loop_closing_->current_keyframe()->id();
//                    auto loop_id = loop_closing_->loop_keyframe()->id();
//                    vector<Keyframe::Ptr> keyframe_window;
//                    keyframeDB_->keyframe_set(keyframe_window, loop_closing_->loop_keyframe()->id(), num_keyframe_);
//                    // gicp
//                    // downsample clouds
//                    PointCloud::Ptr modelcloud;
//                    PointCloud::Ptr modelcloud_downsampled;
//                    modelcloud_downsampled.reset(new PointCloud());
//                    PointCloud::Ptr datacloud;
//                    PointCloud::Ptr datacloud_downsampled;
//                    datacloud_downsampled.reset(new PointCloud());
//                    PointCloud::Ptr transformed;
//                    transformed.reset(new PointCloud());

//                    PointCloud tmp_model;
//                    auto tmpTj = keyframe_window[num_keyframe_-1]->frame()->Twc();
//                    tmp_model = keyframe_window[num_keyframe_-1]->frame()->pointcloud();

//                    for(int i=0; i<num_keyframe_-1; ++i) {
//                        auto tmpTi = keyframe_window[i]->frame()->Twc();
//                        PointCloud tmp_pc = keyframe_window[i]->frame()->pointcloud();
//                        pcl::transformPointCloud(tmp_pc, tmp_pc, (tmpTj.inverse()*tmpTi).matrix());
//                        tmp_model += tmp_pc;
//                    }

//                    PointCloud tmp_data;
//                    tmp_data = current_keyframe->frame()->pointcloud();

//                    modelcloud = tmp_model.makeShared();
//                    datacloud = tmp_data.makeShared();

//                    pcl::VoxelGrid<POINT> vg;
//                    vg.setInputCloud(modelcloud);
//                    vg.setLeafSize (0.1f, 0.1f, 0.1f);
//                    vg.filter(*modelcloud_downsampled);

//    //                vg.setInputCloud(datacloud);
//        //            vg.setLeafSize (0.01f, 0.01f, 0.01f);
//        //            vg.filter(*datacloud_downsampled);

//                    pcl::GeneralizedIterativeClosestPoint<POINT, POINT> gicp;
//                    gicp.setInputSource(datacloud);
//                    gicp.setInputTarget(modelcloud_downsampled);
//                    cerr << modelcloud_downsampled->size() << endl;

//                    auto pc_Ti = loop_closing_->loop_keyframe()->frame()->Twc();
//                    auto pc_Tj = loop_closing_->current_keyframe()->frame()->Twc();
//                    auto pc_Tij = pc_Ti.inverse() * pc_Tj;


//                    gicp.align(*transformed);//, pc_Tij.matrix());

//                    auto gicp_Tij = gicp.getFinalTransformation();
//                    Sophus::SE3f sophus_pc_Tij(gicp_Tij.block<3,3>(0,0), gicp_Tij.block<3,1>(0,3));


//                    cerr << gicp.getFinalTransformation() << endl << endl;

////                    if (fabs(v_t_ij_c.norm() / v_t_ij_p.norm() - 1.0) > fabs(sophus_pc_Tij.log().norm() / v_t_ij_p.norm() - 1.0) ) {
////                        keyframe_window[num_keyframe_-1]->frame()->Twc(keyframe_window[num_keyframe_-2]->frame()->Twc()*sophus_pc_Tij);
////                    }

//                    auto sophus_pc_Tji = sophus_pc_Tij.inverse();
//                    edge = sophus_pc_Tji.cast<double>();
//                }



                Sophus::SE3d edge = loop_closing_->dTji();

                Matrix6x6 loop_info = Matrix6x6::Identity(6, 6);
//                loop_info.block<3,3>(3,3) = loop_info.block<3,3>(3,3)*100;
//                loop_info(0,0) = loop_closing_->tracker()->information()(0,0);
//                loop_info(1,1) = loop_closing_->tracker()->information()(1,1);
//                loop_info(2,2) = loop_closing_->tracker()->information()(2,2);
//                loop_info(3,3) = loop_closing_->tracker()->information()(3,3);
//                loop_info(4,4) = loop_closing_->tracker()->information()(4,4);
//                loop_info(5,5) = loop_closing_->tracker()->information()(5,5);
//                loop_info = loop_info / (loop_closing_->tracker()->f_num_measurement());
//                Matrix6x6 info = loop_closing_->tracker()->information()/(loop_closing_->tracker()->f_num_measurement());

//                info.block<3,3>(0,0) = loop_closing_->tracker()->information().block<3,3>(0,0)/(loop_closing_->tracker()->f_num_measurement());
//                info.block<3,3>(3,3) = loop_closing_->tracker()->information().block<3,3>(3,3)/(loop_closing_->tracker()->f_num_measurement());

                graph_optimizer_.add_edge(edge, loop_info, loop_closing_->current_keyframe()->id(), loop_closing_->loop_keyframe()->id(), true);
                graph_optimizer_.optimize();

                for(auto it=keyframeDB_->begin(); it != keyframeDB_->end(); ++it) {

                    VertexSE3 *pose = graph_optimizer_.estimate_pose2((*it)->id());
                    Sophus::SE3f T = pose->estimate().cast<float> ();

                    (*it)->frame()->Twc(T);
                }

                auto it_j_frame = keyframeDB_->end()-1;
                auto it_i_frame = keyframeDB_->end()-2;
                Tji_ = (*it_j_frame)->frame()->Twc().inverse() * (*it_i_frame)->frame()->Twc();

            }
        }

//        while(1) {
//            if(loop_closing_->is_finished()) break;
//        }

        frameDB_->add(current);

        return true;
    } else {
//        if(keyframeDB_->size() >= num_keyframe_) {
//            cerr << "[System]\t called local_track()" << endl;
//            LocalTracker local_tracker(camera_);
//            local_tracker.tracking(kf_window_->frames());
//        }
        frameDB_->add(current);
    }

//    cout << current->Twc().matrix() << endl;

    return true;

}

}   // namespace dedvo
