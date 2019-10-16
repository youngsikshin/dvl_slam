#pragma once

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/solvers/csparse/linear_solver_csparse.h>
//#include <g2o/solvers/csparse/g2o_csparse_extension_api.h>
// #include "g2o/solvers/csparse/linear_solver_csparse.h"
// #include "g2o/types/sba/types_six_dof_expmap.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/se3quat.h>
//#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>

#include <dedvo/g2o_type_se3sophus.h>
#include <dedvo/Datatypes.h>

namespace dedvo
{
// class
using namespace std;

class GraphOptimizer
{
public:
  typedef shared_ptr<GraphOptimizer> Ptr;
  GraphOptimizer();
  ~GraphOptimizer();

  void add_pose_to_graph(Eigen::Isometry3f& pose, int id);
  void add_pose_to_graph(Sophus::SE3d& pose, int id);
  void add_edge(Eigen::Isometry3f& Tij, int id_i, int id_j);
  void add_edge(Sophus::SE3d& Tij, int id_i, int id_j);
  void add_edge(Sophus::SE3d& Tij, Matrix6x6& H, int id_i, int id_j);
  void add_edge(Sophus::SE3d& Tij, Matrix6x6& H, int id_i, int id_j, bool b_DCS);
  void optimize();
  g2o::VertexSE3* estimate_pose(int id) { return dynamic_cast<g2o::VertexSE3*> (graph_optimizer.vertex(id)); }
  VertexSE3* estimate_pose2(int id) { return dynamic_cast<VertexSE3*> (graph_optimizer.vertex(id)); }

private:
  typedef g2o::BlockSolver_6_3 BlockSolver;
//  typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;
  typedef g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType> LinearSolver;
//  typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;

  g2o::SparseOptimizer graph_optimizer;

  LinearSolver* solver;
  BlockSolver* block_solver;

//  vector<g2o::VertexSE3*> vertices_;
//  vector<g2o::EdgeSE3*> edges_;

};

}
