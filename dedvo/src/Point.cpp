#include <dedvo/Point.h>

namespace dedvo
{
Point::Point()
{

}

Point::Point(Eigen::Vector3f pnt)
    :point(pnt)
{
}

Point::Point(float x, float y, float z)
{
    point << x, y, z;
}

Point::~Point()
{

}

Point& Point::operator=(const Point& lhs)
{
    point = lhs.point;

    return *this;
}

// Point& Point::operator=(const Eigen::Vector3f& lhs)
// {
//  point = lhs;

//  return *this;
// }

}
