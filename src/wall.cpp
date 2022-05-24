#include "wall.h"

Wall::Wall() : line(Point3d(0, 0, 0), Point3d(0, 0, 0)) {}

Wall::Wall(double x1, double y1, double x2, double y2)
    : line(Point3d(x1, y1, 0), Point3d(x2, y2, 0)) {}

Point3d Wall::getNearestPoint(const Point3d &pos) {
  Vector3d relativeEnd, relativePos, relativeEndScal, relativePosScal;
  double dotProduct;
  Point3d nearestPoint;

  relativeEnd = line.b - line.a;
  relativePos = pos - line.a;

  // Scale Both Vectors by the Length of the Wall
  relativeEndScal = relativeEnd;

  relativeEndScal.normalize();

  relativePosScal = relativePos * (1.0 / relativeEnd.length());

  // Compute Dot Product of Scaled Vectors
  dotProduct = relativeEndScal.dot(relativePosScal);

  if (dotProduct < 0.0) // Position of Agent i located before wall's 'start'
    nearestPoint = line.a;
  else if (dotProduct > 1.0) // Position of Agent i located after wall's 'end'
    nearestPoint = line.b;
  else // Position of Agent i located between wall's 'start' and 'end'
    nearestPoint = (relativeEnd * dotProduct) + line.a;

  return nearestPoint;
}

Point3d Wall::getStartPoint() { return line.a; }

Point3d Wall::getEndPoint() { return line.b; }
