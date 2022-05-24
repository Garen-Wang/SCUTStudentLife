#ifndef DSPROJECT_WALL_H
#define DSPROJECT_WALL_H

#include "vecmath.h"

class Wall {
private:
  struct Line {
    Point3d a;
    Point3d b;
    Line(Point3d a, Point3d b) : a(a), b(b) {}
  } line;

public:
  explicit Wall();
  explicit Wall(double x1, double y1, double x2, double y2);
  Point3d getNearestPoint(const Point3d &pos);
  Point3d getStartPoint();
  Point3d getEndPoint();
};

#endif // DSPROJECT_WALL_H
