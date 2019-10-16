#include <dedvo/LoopClosing.h>
#include <dedvo/Conversion.h>


namespace dedvo
{
using namespace std;

LoopClosing::LoopClosing(KeyframeDB::Ptr keyframeDB, shared_ptr<ORBVocabulary> vocabulary, CameraModel::Ptr camera)
    : is_finished_(true), last_loop_id_(0), keyframeDB_(keyframeDB), vocabulary_(vocabulary)
{
    camera_ = camera;
    tracker_.reset(new Tracker());

    num_loop_=0;
}

LoopClosing::~LoopClosing()
{

}

void LoopClosing::run()
{
    is_finished_ = false;

    while(1)
    {
        if(check_new_keyframe()) {
            is_finished_ = false;
            if(detect_loop()) {
//                if( loop_constraint) {
//                if(num_loop_ < 3)
                  correct_loop();
            }
            is_finished_ = true;
        }

        if (check_finish())
          break;

        usleep(10);
    }
}

bool LoopClosing::run_without_thread()
{
    is_finished_ = false;

    bool b_detect_loop = false;

    if(check_new_keyframe()) {
        is_finished_ = false;
        b_detect_loop = detect_loop();
//        if(detect_loop()) {
//                if( loop_constraint) {
//                if(num_loop_ < 3)
//              correct_loop();
//        }
        is_finished_ = true;
    }

//    if (check_finish())
//      return b_detect_loop;

    return b_detect_loop;
}


bool LoopClosing::check_new_keyframe()
{
    unique_lock<std::mutex> lock(mtx_kf_queue_);
    return (!keyframe_queue_.empty());
}

bool LoopClosing::detect_loop()
{
    {
        unique_lock<std::mutex> lock(mtx_kf_queue_);
        current_keyframe_ = keyframe_queue_.front();
        keyframe_queue_.pop_front();

        keyframes_.push_back(current_keyframe_);
    }

    if(current_keyframe_->id() < last_loop_id_ + 10) {
        return false;
    }

    vector<Keyframe::Ptr> connected_keyframe;
    keyframeDB_->connected_keyframe(connected_keyframe, 6);

    const DBoW2::BowVector &current_bow_vec = current_keyframe_->BoW_vec();
    float min_si =1.0;

    for (size_t i=0; i<connected_keyframe.size(); ++i) {
        Keyframe::Ptr keyframe_i = connected_keyframe[i];

        const DBoW2::BowVector &bow_vec = keyframe_i->BoW_vec();

        float si = vocabulary_->score(current_bow_vec, bow_vec);

        if (si < min_si)  min_si = si;
    }

    vector<Keyframe::Ptr> candidate_frame = detect_loop_candidate(current_keyframe_, min_si);
    cerr << "[LoopClosing]\t Number of candidate keyframe size : " << candidate_frame.size() << endl;

    if (candidate_frame.empty())
        return false;


    cv::namedWindow("query image");
    cv::imshow("query image", current_keyframe_->frame()->level(1));

    float max_score = 0.0;

    for(auto it = candidate_frame.begin(); it != candidate_frame.end(); ++it) {

        if ( (*it)->loop_score() > max_score ) {
            max_score = (*it)->loop_score();
            loop_keyframe_ = *it;
        }

    }
    cv::namedWindow("DB image");
    cv::imshow("DB image", loop_keyframe_->frame()->level(1));
    cv::waitKey(10);

    last_loop_id_ = current_keyframe_->id();
    num_loop_++;
    cerr << "[LoopClosing]\t number of candidate frame : " << candidate_frame.size() << endl;

    return true;
  //            // loop detect test

  //            if (keyframeDB_->size() > num_keyframe_+10) {
  //                float min_si = 1.0;

  //                for(auto n_kf_it = keyframe_window.begin(); n_kf_it != keyframe_window.end()-5; ++n_kf_it) {
  //                    float si = vocabulary_->score(current_keyframe->frame()->BoW_vec(), (*n_kf_it)->frame()->BoW_vec());

  //                    min_si = si < min_si ? si : min_si;
  //                }


  //                int kDB_idx = 0;
  //                vector< pair<int, float> > kDB_score;
  //                float best_score = min_si;

  //                for(auto kDB_it = keyframeDB_->begin(); kDB_it != keyframeDB_->end()-num_keyframe_-10; ++kDB_it, ++kDB_idx) {
  //                    float si = vocabulary_->score(current_keyframe->frame()->BoW_vec(), (*kDB_it)->frame()->BoW_vec());
  //                    kDB_score.push_back(make_pair(kDB_idx, si));
  //                    best_score = si > best_score ? si : best_score;
  //                }



  //                float retain_score = 0.8 * best_score;
  //                auto kDB = keyframeDB_->keyframeDB();

  //                if(kDB_score.size() > 5 ) { //&& best_score > 0.07 && best_score > min_si*2.0
  //                    int matched_id = -1;
  //                    float best_matched_score = -1.0;

  //                    for(size_t i=2; i<kDB_score.size()-3; ++i) {
  //                      if(kDB_score[i].second > min_si && kDB_score[i-1].second > min_si && kDB_score[i+1].second > min_si) {
  //                        if(kDB_score[i].second > retain_score && kDB_score[i-1].second > retain_score && kDB_score[i+1].second > retain_score) {

  //                            if(kDB_score[i].second > best_matched_score) {
  //                              matched_id = kDB_score[i].first;
  //                              best_matched_score = kDB_score[i].second;
  //                            }
  //                        }
  //                      }
  //                    }

  //                    if(matched_id != -1) {
  //                        cerr << matched_id << "/" << kDB.size() << endl;
  //                        cv::namedWindow("query image");
  //                        cv::imshow("query image", current_keyframe->frame()->level(1));
  //                        cv::namedWindow("DB image");
  //                        cv::imshow("DB image", kDB[matched_id]->frame()->level(1));
  //                        cv::waitKey(500);
  //                    }
  //                }


  //                cerr << "[System]\t Minimum similarity score is " << min_si << endl;
  //                cerr << "[System]\t Best similarity score is " << best_score << endl;

  //            }

}

vector<Keyframe::Ptr> LoopClosing::detect_loop_candidate(Keyframe::Ptr keyframe, float min_si)
{
    vector<Keyframe::Ptr> connected_keyframe;
    keyframeDB_->connected_keyframe(connected_keyframe, 6);

    list<Keyframe::Ptr> keyframe_list_sharing_words;

    for(DBoW2::BowVector::const_iterator it=keyframe->BoW_vec().begin(); it != keyframe->BoW_vec().end(); ++it)
    {
        list<Keyframe::Ptr> &lKFs = keyframeDB_->inverted_file()[it->first];

        for(list<Keyframe::Ptr>::iterator lit=lKFs.begin(); lit!=lKFs.end(); ++lit)
        {
            Keyframe::Ptr keyframe_i=*lit;

            if(keyframe_i->id() == current_keyframe_->id()) continue;

            if(keyframe_i->loop_query()!=keyframe->id())
            {
                keyframe_i->num_loop_word(0);
//                if(!spConnectedKeyFrames.count(pKFi))
//                {
                    keyframe_i->loop_query(keyframe->id());
                    keyframe_list_sharing_words.push_back(keyframe_i);
//                }
            }
            keyframe_i->inc_num_loop_word();
        }
    }

    if(keyframe_list_sharing_words.empty())
        return vector<Keyframe::Ptr>();

    list< pair<float, Keyframe::Ptr> > list_score_keyframe;

    int max_common_words = 0;

    for ( auto it = keyframe_list_sharing_words.begin(); it != keyframe_list_sharing_words.end(); ++it) {
        if ((*it)->num_loop_word() > max_common_words) {
            max_common_words = (*it)->num_loop_word();
        }
    }

    int min_common_word = static_cast<int> (0.75*max_common_words);
    int num_score =0;

    cerr << "[LoopClosing]\t min common word : " << min_common_word << endl;
    cerr << "[LoopClosing]\t list sharing words size : " << keyframe_list_sharing_words.size() << endl;


    for ( auto it = keyframe_list_sharing_words.begin(); it != keyframe_list_sharing_words.end(); ++it) {

        Keyframe::Ptr keyframe_i = *it;

        if( current_keyframe_->id() == keyframe_i->id())  continue;

        if ( keyframe_i->num_loop_word() > min_common_word ) {
            ++num_score;

            float si = vocabulary_->score(current_keyframe_->BoW_vec(), keyframe_i->BoW_vec());

            keyframe_i->loop_score(si);

            if ( si >= min_si )
                list_score_keyframe.push_back(make_pair(si, keyframe_i));
        }

    }

    cerr << "[LoopClosing]\t score and keyframe size : " << list_score_keyframe.size() << endl;

    if ( list_score_keyframe.empty() )
        return vector<Keyframe::Ptr>();

    list<pair<float, Keyframe::Ptr>> list_accept_score_keyframe;
    float best_accept_score = min_si;

    for ( auto it = list_score_keyframe.begin(); it != list_score_keyframe.end(); ++it) {
        Keyframe::Ptr keyframe_i = it->second;

        if(keyframe_i->id() == current_keyframe_->id()) continue;
        if(keyframe_i->first_connection()) continue;

        vector<Keyframe::Ptr> neighbor_frame;

        neighbor_frame.push_back(keyframe_i->parent());
        neighbor_frame.push_back(keyframe_i->child());

        float best_score = it->first;
        float accept_score = it->first;
        Keyframe::Ptr best_keyframe = keyframe_i;

        for ( auto nit = neighbor_frame.begin(); nit != neighbor_frame.end(); ++nit) {
            Keyframe::Ptr keyframe2 = *nit;

            if( keyframe2->loop_query() == current_keyframe_->id() && keyframe2->num_loop_word() > min_common_word) {
                accept_score += keyframe2->loop_score();

                if ( keyframe2->loop_score() > best_score ) {
                    best_keyframe = keyframe2;
                    best_score = keyframe2->loop_score();
                }
            }
        }

        list_accept_score_keyframe.push_back(make_pair(accept_score, best_keyframe));
        if(accept_score > best_score)
            best_accept_score = accept_score;
    }

    float min_score_retain = 0.75 * best_accept_score;
    set<Keyframe::Ptr> already_added_keyframe;
    vector<Keyframe::Ptr> loop_candidate;

    for (auto it = list_accept_score_keyframe.begin(); it != list_accept_score_keyframe.end(); ++it) {
        if ( it->first > min_score_retain ) {
            Keyframe::Ptr keyframe_i = it->second;

            if (current_keyframe_->id() - keyframe_i->id() < 50) continue;

            if(!already_added_keyframe.count(keyframe_i)) {
                loop_candidate.push_back(keyframe_i);
                already_added_keyframe.insert(keyframe_i);
            }
        }
    }

    return loop_candidate;
}

void LoopClosing::correct_loop()
{
    graph_optimizer_.reset(new GraphOptimizer);

    bool initialized = false;

    unique_lock<mutex> ulock(keyframeDB_->mtx_DB);

    auto it_end = keyframeDB_->end();
    for(auto it=keyframeDB_->begin(); it != it_end; ++it) {
        Eigen::Isometry3f estimate_pose;
        Converter::sophus_se3f_to_eigen_isometry3f((*it)->frame()->Twc(), estimate_pose);
//        graph_optimizer_->add_pose_to_graph(estimate_pose, (*it)->id());

        Sophus::SE3d pose = (*it)->frame()->Twc().cast<double>();
        graph_optimizer_->add_pose_to_graph(pose, (*it)->id());
//        cerr << (*it)->id() << endl;
    }

    for(auto it=keyframeDB_->begin(); it != it_end-1; ++it) {

        Sophus::SE3f Tij = (*it)->frame()->Twc().inverse()*(*(it+1))->frame()->Twc();
        Eigen::Isometry3f measured_Tij;
        Converter::sophus_se3f_to_eigen_isometry3f(Tij, measured_Tij);

//        cerr << (*(it))->id() << ", " << (*(it+1))->id() << " / " << keyframeDB_->size() << endl;
//        graph_optimizer_->add_edge(measured_Tij, (*(it))->id(), (*(it+1))->id());

        Sophus::SE3d dTij = Tij.cast<double>();
        graph_optimizer_->add_edge(dTij, (*(it))->id(), (*(it+1))->id());

    }

//    for(auto it=keyframeDB_->begin(); it != keyframeDB_->end(); ++it) {
//        if(!initialized) {
//            Eigen::Isometry3f estimate_pose;
//            sophus_se3f_to_eigen_isometry3f((*it)->frame()->Twc(), estimate_pose);
//            graph_optimizer_->add_pose_to_graph(estimate_pose, (*it)->id());
//            cerr << (*it)->id() << endl;

//            initialized = true;

////            continue;
//        } else {

//            Eigen::Isometry3f estimate_pose;
//            sophus_se3f_to_eigen_isometry3f((*it)->frame()->Twc(), estimate_pose);
//            graph_optimizer_->add_pose_to_graph(estimate_pose, (*it)->id());

//            Sophus::SE3f Tij = (*(it-1))->frame()->Twc().inverse()*(*it)->frame()->Twc();
//            Eigen::Isometry3f measured_Tij;
//            sophus_se3f_to_eigen_isometry3f(Tij, measured_Tij);
////            cerr << (*it)->id() << endl;
////            cerr << Tij.matrix() << endl;
////            cerr << measured_Tij.matrix() << endl;

//            cerr << "id i: " << (*(it-1))->id() << " id j: " << (*it)->id() << endl;
//            graph_optimizer_->add_edge(measured_Tij, (*(it-1))->id(), (*it)->id());
//        }
////        cerr << (*it)->id() << endl;
//    }

    tracker_->match(loop_keyframe_, current_keyframe_->frame(), Tji_);
    Tij_ = Tji_.inverse();

    Sophus::SE3d dTji = Tji_.cast<double>();
    Sophus::SE3d dTij = Tij_.cast<double>();

    Eigen::Isometry3f measured_Tij;
    Converter::sophus_se3f_to_eigen_isometry3f(Tij_, measured_Tij);

//    graph_optimizer_->add_edge(measured_Tij, loop_keyframe_->id(), current_keyframe_->id());
    graph_optimizer_->add_edge(dTji, current_keyframe_->id(), loop_keyframe_->id());

//    cerr << loop_keyframe_->id() << ", " << current_keyframe_->id() << endl;
//    cerr << Tji.matrix() << endl;

    graph_optimizer_->optimize();

    for(auto it=keyframeDB_->begin(); it != it_end; ++it) {
//        g2o::VertexSE3 *pose = graph_optimizer_->estimate_pose((*it)->id());
//        cerr << pose->estimate().matrix() << endl;
//        auto eigen_T = pose->estimate().matrix();
//        auto f_eigen_T = eigen_T.cast<float>();
//        Sophus::SE3f T(f_eigen_T.block<3,3>(0,0), f_eigen_T.block<3,1>(0,3));

        VertexSE3 *pose = graph_optimizer_->estimate_pose2((*it)->id());
        Sophus::SE3f T = pose->estimate().cast<float> ();

        (*it)->frame()->Twc(T);
    }
}

void LoopClosing::insert_keyframe(Keyframe::Ptr keyframe)
{
    unique_lock<mutex> lock(mtx_kf_queue_);
    if(keyframe->id() != 0)
        keyframe_queue_.push_back(keyframe);
}

bool LoopClosing::loop_constraint()
{
    bool status;
    status = tracker_->match(loop_keyframe_, current_keyframe_->frame(), Tji_);
    Tij_ = Tji_.inverse();

    bool status2;
    Sophus::SE3f Tij = Tij_;
    status2 = tracker_->match(current_keyframe_, loop_keyframe_->frame(), Tij);

    auto diff_tr = (Tji_*Tij).log().head(3).norm();
    auto diff_rot = (Tji_*Tij).log().tail(3).norm();

    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;
    cerr << diff_tr << ", " << diff_rot << endl;

//    if( diff_tr < 0.04 && diff_rot < 0.007 && status)
    if( diff_tr < 0.007 && diff_rot < 0.007 && status)
        return true;
    else
        return false;

    return status;
}

void LoopClosing::request_finish()
{
    unique_lock<std::mutex> lock(mtx_request_finish_);
    request_finish_ = true;
}

bool LoopClosing::check_finish()
{
    unique_lock<std::mutex> lock(mtx_request_finish_);
    return request_finish_;
}

void LoopClosing::finish()
{
    unique_lock<std::mutex> lock(mtx_request_finish_);
    is_finished_ = true;
}

bool LoopClosing::is_finished()
{
    unique_lock<std::mutex> lock(mtx_request_finish_);
    return is_finished_;
}

} // namespace dedvo
