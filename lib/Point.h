/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: Point.h
 */

#ifndef WAVEFRONT_POINT_H
#define WAVEFRONT_POINT_H

#include <string>


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
