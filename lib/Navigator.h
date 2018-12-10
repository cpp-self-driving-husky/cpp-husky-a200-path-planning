// Allen Kim | Mar. 14, 2017
// CS 521 (Winter, 2017)
// Project 4: Path Planning & Navigation
// Navigator.h

#ifndef WAVEFRONT_NAVIGATOR_H
#define WAVEFRONT_NAVIGATOR_H

#include <vector>
#include <string>
#include "Pilot.h"
#include "Coordinates.h"

class Navigator {
public:
    Navigator();
    Navigator(std::string filename);
    Navigator(Point *goal);
    virtual ~Navigator();
    void setMyPilot(Pilot *newPilot);
    virtual void processWaypoints();
    void navPFields();

protected:
    std::list<Point> points;
    Pilot *myPilot;

private:

};

#endif // WAVEFRONT_NAVIGATOR_H
