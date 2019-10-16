#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/se3quat.h>

#include <sophus/se3.hpp>

#include <Eigen/Core>

namespace dedvo {

class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSE3();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setToOriginImpl() {
        _estimate = Sophus::SE3d();
      }

      virtual void oplusImpl(const double* update_)
      {
        Eigen::Map< Eigen::Matrix<double, 6, 1> > update(const_cast<double*>(update_));

        setEstimate( Sophus::SE3d::exp(update) * estimate());
      }
};


class EdgeSE3 : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3, VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* _to = static_cast<const VertexSE3*>(_vertices[1]);

        Sophus::SE3d error_= _from->estimate().inverse() * _to->estimate() * _inverseMeasurement;
        _error = error_.log();
    }

    void linearizeOplus()
    {
        const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);

        _jacobianOplusXj = _from->estimate().inverse().Adj();
        _jacobianOplusXi = -_jacobianOplusXj;
    }


    virtual void setMeasurement(const Sophus::SE3d& m)
    {
        _measurement = m;
        _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* m)
    {
        Eigen::Map<const g2o::Vector6d> v(m);
        setMeasurement(Sophus::SE3d::exp(v));
        return true;
    }

    virtual bool setMeasurementFromState()
    {
        const VertexSE3* from = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* to   = static_cast<const VertexSE3*>(_vertices[1]);
        Sophus::SE3d delta = from->estimate().inverse() * to->estimate();
        setMeasurement(delta);
        return true;
    }

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}

    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
    {
        VertexSE3 *_from = static_cast<VertexSE3*>(_vertices[0]);
        VertexSE3 *_to   = static_cast<VertexSE3*>(_vertices[1]);

        if (from.count(_from) > 0)
            _to->setEstimate(_from->estimate() * _measurement);
        else
            _from->setEstimate(_to->estimate() * _inverseMeasurement);
    }

protected:
    Sophus::SE3d _inverseMeasurement;
};

} // namespace dedvo
