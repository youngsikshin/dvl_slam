#include <dedvo/g2o_type_se3sophus.h>

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

namespace dedvo {

G2O_USE_TYPE_GROUP(slam3d);

G2O_REGISTER_TYPE_GROUP(se3sophus);

G2O_REGISTER_TYPE(VERTEX_SE3_SOPHUS:EXPMAP, VertexSE3);
G2O_REGISTER_TYPE(EDGE_SE3_SOPHUS:EXPMAP, EdgeSE3);

VertexSE3::VertexSE3() : g2o::BaseVertex<6, Sophus::SE3d>()
{
    _marginalized=false;
}

bool VertexSE3::write(std::ostream& os) const
{
    // TODO
    assert(false);
    return false;
//     Sim3 cam2world(estimate().inverse());
//     Vector7d lv=cam2world.log();
//     for (int i=0; i<7; i++){
//       os << lv[i] << " ";
//     }
//     for (int i=0; i<2; i++)
//     {
//       os << _focal_length[i] << " ";
//     }
//     for (int i=0; i<2; i++)
//     {
//       os << _principle_point[i] << " ";
//     }
//     return os.good();
}

bool VertexSE3::read(std::istream& is)
{
    // TODO
    assert(false);
    return false;
//     Vector7d cam2world;
//     for (int i=0; i<6; i++){
//       is >> cam2world[i];
//     }
//     is >> cam2world[6];
// //    if (! is) {
// //      // if the scale is not specified we set it to 1;
// //      std::cerr << "!s";
// //      cam2world[6]=0.;
// //    }
//
//     for (int i=0; i<2; i++)
//     {
//       is >> _focal_length[i];
//     }
//     for (int i=0; i<2; i++)
//     {
//       is >> _principle_point[i];
//     }
//
//     setEstimate(Sim3(cam2world).inverse());
//     return true;
}


EdgeSE3::EdgeSE3() :
  g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3, VertexSE3>()
{
}

bool EdgeSE3::write(std::ostream& os) const
{
  // TODO
  assert(false);
  return false;
//     Sim3 cam2world(measurement().inverse());
//     Vector7d v7 = cam2world.log();
//     for (int i=0; i<7; i++)
//     {
//       os  << v7[i] << " ";
//     }
//     for (int i=0; i<7; i++)
//       for (int j=i; j<7; j++){
//         os << " " <<  information()(i,j);
//     }
//     return os.good();
}

bool EdgeSE3::read(std::istream& is)
{
  // TODO
  assert(false);
  return false;
//     Vector7d v7;
//     for (int i=0; i<7; i++){
//       is >> v7[i];
//     }
//
//     Sim3 cam2world(v7);
//     setMeasurement(cam2world.inverse());
//
//     for (int i=0; i<7; i++)
//       for (int j=i; j<7; j++)
//       {
//         is >> information()(i,j);
//         if (i!=j)
//           information()(j,i)=information()(i,j);
//       }
//     return true;
}


} // namespace dedvo
