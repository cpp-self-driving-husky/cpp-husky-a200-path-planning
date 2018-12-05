/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: Coord.h
 */

#ifndef WAVEFRONT_COORD_H
#define WAVEFRONT_COORD_H

#include "Point.h"

/**
 * A Coord stores the latitude and longitude of a location.
 */

class Coord {

public:
    Coord();
    Coord(double longitude, double latitude);
    virtual ~Coord() = default;
    double getLong() const;
    void setLong(double longi);
    double getLat() const;
    void setLat(double lat);
    bool equals(Coord coord);
    void printCoord();

protected:

private:
    double longitude;
    double latitude;
};

#endif //WAVEFRONT_COORD_H
