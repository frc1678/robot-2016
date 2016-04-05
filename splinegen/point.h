#ifndef SPLINE_POINT_H_
#define SPLINE_POINT_H_

#include <cmath>
#include "muan/unitscpp/unitscpp.h"
#include <type_traits>

namespace spline
{
template<typename T>
class Point {
 public:
  Point() {}
  Point(T x, T y) : x(x), y(y) {}
  T x, y;
  T Magnitude() {
    return T(std::sqrt(x() * x() + y() * y()));
  }
  Angle Direction() {
    return std::atan2(y(), x()) * rad;
  }
};

template<typename T>
Point<T> operator+(const Point<T>& p1, const Point<T>& p2) {
  return Point<T>( p1.x + p2.x, p1.y + p2.y );
}

template<typename T>
Point<T> operator-(const Point<T>& p1, const Point<T>& p2) {
  return Point<T>( p1.x - p2.x, p1.y - p2.y );
}

template<typename T, typename S>
auto operator*(const S& scalar, const Point<T>& p) {
  using R = std::remove_cv_t<decltype(scalar * p.x)>;
  return Point<R>(scalar * p.x, scalar * p.y);
}

template<typename T, typename S>
auto operator*(const Point<T>& p, const S& scalar) {
  using R = std::remove_cv_t<decltype(p.x * scalar)>;
  return Point<R>(p.x * scalar, p.y * scalar);
}

template<typename T, typename S>
auto operator/(const Point<T>& p, const S& scalar) {
  using R = std::remove_cv_t<decltype(p.x / scalar)>;
  return Point<R>(p.x / scalar, p.y / scalar);
}

}

#endif /* SPLINE_POINT_H_ */
