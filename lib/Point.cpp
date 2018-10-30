/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: Point.cpp
 */

#include <iostream>
#include "Point.h"

Point::Point() : xCoord(0), yCoord(0) {
}

Point::Point(double x, double y) : xCoord(x), yCoord(y) {
}

Point Point::promptUser(std::string message) {
    double x, y;
    std::cout << message << std::endl;
    std::cout << "     %$    x = ";
    std::cin >> x;
    std::cout << "     %$    y = ";
    std::cin >> y;
    return Point(x, y);
}

double Point::getX() {
    return xCoord;
}

double Point::getY() {
    return yCoord;
}
