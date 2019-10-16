#include <iostream>
#include <dedvo/GraphOptimizer.h>

using namespace std;

namespace dedvo
{

GraphOptimizer::GraphOptimizer()
{
  cerr << "[GraphOptimizer]\t Enter the constructor..." << endl;
  solver = new LinearSolver();
  block_solver = new BlockSolver(solver);
//  g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
  g2o::OptimizationAlgorithmGaussNewton* algorithm = new g2o::OptimizationAlgorithmGaussNewton(block_solver);

  graph_optimizer.setAlgorithm(algorithm);
  graph_optimizer.setVerbose(false);
}

GraphOptimizer::~GraphOptimizer()
{
    cerr << "[GraphOptimize] Save a optimized graph result" << endl;
//    graph_optimizer.save("result.g2o");
}

void GraphOptimizer::add_pose_to_graph(Eigen::Isometry3f& pose, int id)
{
    g2o::VertexSE3 *v;
    v = new g2o::VertexSE3;
//    g2o::VertexSE3Expmap *v;
//    v = new g2o::VertexSE3Expmap();

    Eigen::Isometry3d pose_estimate;
    pose_estimate = pose.cast<double>();
//    pose_estimate = pose.rotation();
//    pose_estimate.translation() = pose.translation();
//    g2o::Isometry3D g2o_pose;//(pose_estimate);
//    g2o_pose = pose_estimate.matrix();

//    g2o::SE3Quat se3quat_pose(pose_estimate.rotation(), pose_estimate.translation());

    v->setEstimate(pose_estimate);
    v->setId(id);

    if(id == 0) {
//        v->setFixed(true);
    }
//    else v->setFixed(false);

//    v->setMarginalized(false);

    graph_optimizer.addVertex(v);
//    vertices_.push_back(v);

}

void GraphOptimizer::add_edge(Eigen::Isometry3f& Tij, int id_i, int id_j)
{
    g2o::EdgeSE3 *e;
    e = new g2o::EdgeSE3;
//    g2o::EdgeSE3Expmap *e;
//    e = new g2o::EdgeSE3Expmap();

    // retrieve vertex pointers from graph with id's
//    g2o::OptimizableGraph::Vertex * pose_i_vertex
//        = dynamic_cast<g2o::OptimizableGraph::Vertex*> (graph_optimizer.vertices()[id_i]);

//    g2o::OptimizableGraph::Vertex * pose_j_vertex
//        = dynamic_cast<g2o::OptimizableGraph::Vertex*> (graph_optimizer.vertices()[id_j]);

//    // error check vertices and associate them with the edge
//    assert(pose_i_vertex!=NULL);
//    assert(pose_j_vertex->dimension() == 6);
//    e->vertices()[0] = pose_i_vertex;

//    assert(pose_i_vertex!=NULL);
//    assert(pose_j_vertex->dimension() == 6);
//    e->vertices()[1] = pose_j_vertex;
//    e->setVertex(0, dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertex(id_i)));
//    e->setVertex(1, dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertex(id_j)));

    e->resize(2);
    g2o::VertexSE3* prev = dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertex(id_i));
    g2o::VertexSE3* cur  = dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertex(id_j));
    e->setVertex(0, prev);
    e->setVertex(1, cur);

//    Eigen::Isometry3d est = vv->estimate();
//    cerr << est.matrix() << endl;
//     add information matrix
    Eigen::Matrix<double, 6, 6> Lambda;
    Lambda.setIdentity();
    Lambda.block<3,3> (0,0) = Lambda.block<3,3> (0,0)*10000.0;
    Lambda.block<3,3> (3,3) = Lambda.block<3,3> (3,3)*50000.0;

    // set the observation and imformation matrix
    Eigen::Isometry3d dTij = prev->estimate().inverse() * cur->estimate();

    g2o::Isometry3D g2o_dTij;//(dTij);
    g2o_dTij = dTij.rotation();
    g2o_dTij.translation() = dTij.translation();

    cerr << g2o_dTij.matrix() << endl;

//    g2o::SE3Quat se3quat_Tij(dTij.rotation(), dTij.translation());

    cerr << dTij.matrix() << endl;
    e->setMeasurement(dTij);

//    e->setInformation(Lambda);



    // finally add the edge to the graph
    if(!graph_optimizer.addEdge(e))
    {
        assert(false);
    }

//    edges_.push_back(e);
}

void GraphOptimizer::add_pose_to_graph(Sophus::SE3d &pose, int id)
{
    VertexSE3 *v;
    v = new VertexSE3;

    v->setEstimate(pose);
    v->setId(id);

    if (id == 0)  v->setFixed(true);
    v->setMarginalized(false);

    graph_optimizer.addVertex(v);
}

void GraphOptimizer::add_edge(Sophus::SE3d& Tij, int id_i, int id_j)
{
    EdgeSE3 *e;

    e = new EdgeSE3;

    e->resize(2);
    VertexSE3* prev = dynamic_cast<VertexSE3*>(graph_optimizer.vertex(id_i));
    VertexSE3* cur  = dynamic_cast<VertexSE3*>(graph_optimizer.vertex(id_j));
    e->setVertex(0, prev);
    e->setVertex(1, cur);

    Eigen::Matrix<double, 6, 6> Lambda;
    Lambda.setIdentity();
//    Lambda.block<3,3> (0,0) = Lambda.block<3,3> (0,0)*10000.0;
//    Lambda.block<3,3> (3,3) = Lambda.block<3,3> (3,3)*40000.0;

    e->setMeasurement(Tij);
    e->setInformation(Lambda);

    string str_robust_kernel("DCS");
    g2o::AbstractRobustKernelCreator* creator = g2o::RobustKernelFactory::instance()->creator(str_robust_kernel);
    if (! creator) {
      cerr << str_robust_kernel << " is not a valid robust kernel" << endl;
      return;
    }

//    e->setRobustKernel(creator->construct());

    graph_optimizer.addEdge(e);
}

void GraphOptimizer::add_edge(Sophus::SE3d& Tij, Matrix6x6& H, int id_i, int id_j)
{
  EdgeSE3 *e;

  e = new EdgeSE3;

  e->resize(2);
  VertexSE3* prev = dynamic_cast<VertexSE3*>(graph_optimizer.vertex(id_i));
  VertexSE3* cur  = dynamic_cast<VertexSE3*>(graph_optimizer.vertex(id_j));
  e->setVertex(0, prev);
  e->setVertex(1, cur);

  Eigen::Matrix<double, 6, 6> Lambda = H.cast<double>();
//  Lambda.setIdentity();
//    Lambda.block<3,3> (0,0) = Lambda.block<3,3> (0,0)*10000.0;
//    Lambda.block<3,3> (3,3) = Lambda.block<3,3> (3,3)*40000.0;

  e->setMeasurement(Tij);
  e->setInformation(Lambda);

  string str_robust_kernel("DCS");
  g2o::AbstractRobustKernelCreator* creator = g2o::RobustKernelFactory::instance()->creator(str_robust_kernel);
  if (! creator) {
    cerr << str_robust_kernel << " is not a valid robust kernel" << endl;
    return;
  }

//    e->setRobustKernel(creator->construct());

  graph_optimizer.addEdge(e);
}

void GraphOptimizer::add_edge(Sophus::SE3d& Tij, Matrix6x6& H, int id_i, int id_j, bool b_DCS)
{
  EdgeSE3 *e;

  e = new EdgeSE3;

  e->resize(2);
  VertexSE3* prev = dynamic_cast<VertexSE3*>(graph_optimizer.vertex(id_i));
  VertexSE3* cur  = dynamic_cast<VertexSE3*>(graph_optimizer.vertex(id_j));
  e->setVertex(0, prev);
  e->setVertex(1, cur);

  Eigen::Matrix<double, 6, 6> Lambda = H.cast<double>();
//  Lambda.setIdentity();
//    Lambda.block<3,3> (0,0) = Lambda.block<3,3> (0,0)*10000.0;
//    Lambda.block<3,3> (3,3) = Lambda.block<3,3> (3,3)*40000.0;

  e->setMeasurement(Tij);
  e->setInformation(Lambda);

  string str_robust_kernel("DCS");
  g2o::AbstractRobustKernelCreator* creator = g2o::RobustKernelFactory::instance()->creator(str_robust_kernel);
  if (! creator) {
    cerr << str_robust_kernel << " is not a valid robust kernel" << endl;
    return;
  }

  if(b_DCS) {
    //kitti01 : 5.0
    //kitti06 : 5.0
    //kitti05 : 100.0

    e->setRobustKernel(creator->construct());
    e->robustKernel()->setDelta(5.0);
  }

  graph_optimizer.addEdge(e);
}

void GraphOptimizer::optimize()
{
//    graph_optimizer.save("before_result.g2o");
    graph_optimizer.initializeOptimization();
    graph_optimizer.setVerbose(true);
    graph_optimizer.optimize(10);
    cerr << "Verificatoin!!!! " << graph_optimizer.verifyInformationMatrices() << endl;

//    ofstream fileOutputStream;
//    string outFilename("result.g2o");
//    if (outFilename != "-") {
//        cerr << "Writing into " << outFilename << endl;
//        fileOutputStream.open(outFilename.c_str());
//    } else {
//      cerr << "writing to stdout" << endl;
//    }

//    string vertexTag = g2o::Factory::instance()->tag(vertices_[0]);
//    string edgeTag = g2o::Factory::instance()->tag(edges_[0]);

//    ostream& fout = outFilename != "-" ? fileOutputStream : cout;

//    for (size_t i = 0; i < vertices_.size(); ++i) {
//      g2o::VertexSE3* v = vertices_[i];
//      fout << vertexTag << " " << v->id() << " ";
//      v->write(fout);
//      fout << endl;
//    }

//    for (size_t i = 0; i < edges_.size(); ++i) {
//      g2o::EdgeSE3* e = edges_[i];
//      g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e->vertex(0));
//      g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e->vertex(1));
//      fout << edgeTag << " " << from->id() << " " << to->id() << " ";
//      e->write(fout);
//      fout << endl;
//    }
}

} // namespace de_dvo2
