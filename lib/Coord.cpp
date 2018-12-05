//
// Created by ak on 12/04/18.
//

#include "Coord.h"

Coord::Coord() : longitude(0), latitude(0) {
}

Coord::Coord(double longi, double lat) : longitude(longi), latitude(lat) {
}

double Coord::getLong() const {
    return longitude;
}

void Coord::setLong(double longi) {
    longitude = longi;
}

double Coord::getLat() const {
    return latitude;
}

void Coord::setLat(double lat) {
    latitude = lat;
}

bool Coord::equals(Coord coord) {
    return (longitude == coord.longitude && latitude == coord.latitude);
}

void Coord::printCoord() {

}



