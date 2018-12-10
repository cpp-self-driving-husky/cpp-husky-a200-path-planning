// Allen Kim | Mar. 14, 2017
// CS 521 (Winter, 2017)
// Project 4: Path Planning & Navigation
// Navigator.cpp

#include <libplayerc++/playerc++.h>
#include <fstream>

#include "Navigator.h"

using namespace std;
using namespace PlayerCc;

Navigator::Navigator() {
}

/**
 * Constructor that reads in waypoints from a text file. Example waypoints file:
 ----Start of file----
 -2.1  8
 5  4
 10  -6.5
 ----End of file----
 *
 * @param filename
 */
Navigator::Navigator(string filename) {
    ifstream ifs;
    ifs.open(filename.c_str(), ifstream::in);
    double x, y;
    while (ifs >> x >> y) {
        Point newPoint(x, y);
        points.push_back(newPoint);
        cout << "NAVIGATOR: added (" << x << ", " << y << ")" << endl;
    }
    cout << endl << endl;
}

/**
 * Constructor for a single goal waypoint
 *
 * @param goal
 */
Navigator::Navigator(Point *goal) {
    points.push_back(*goal);
    cout << "NAVIGATOR: added (" << goal->getX() << ", " << goal->getY() << ")" << endl;
}

Navigator::~Navigator() {
    delete &points;
//    delete myPilot;
}

void Navigator::setMyPilot(Pilot *newPilot) {
    myPilot = newPilot;
}

/**
 * Project 2 - feeds the waypoints one by one to the Pilot.
 */
void Navigator::processWaypoints() {
    for (auto pt : points) {
        myPilot->setGoal(&pt);
        myPilot->gotoGoal();
        sleep(5);
    }
    cout << "NAVIGATOR: processed all waypoints" << endl;
}

/**
 * Project 3 - tells the Pilot to use the potential fields method to go to goal.
 */
void Navigator::navPFields() {
    myPilot->setGoal(&points.front());
    myPilot->seekGoalWithObstacles();
    return;
}


