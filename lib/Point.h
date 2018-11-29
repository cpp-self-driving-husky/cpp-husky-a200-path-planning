/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: Point.h
 */

#ifndef WAVEFRONT_POINT_H
#define WAVEFRONT_POINT_H

#include <string>

/**
 * A Point stores the (x, y) coordinates of a location. The origin is the top left
 * corner of the map. Positive x direction is to the right; positive y direction is
 * downward (similar to OccGrid).
 */

class Point {
public:
    Point();
    Point(double x, double y);
    virtual ~Point() = default;
    double getX();
    double getY();
    static Point promptUser(std::string message);

protected:

private:
    double xCoord = 0.0;
    double yCoord = 0.0;
};

#endif // WAVEFRONT_POINT_H
