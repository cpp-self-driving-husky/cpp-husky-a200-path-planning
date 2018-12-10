/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: Coordinates.cpp
 */

#include <iostream>
#include <sstream>
#include <string>

#include "Coordinates.h"


Point::Point() : xCoord(0), yCoord(0) {
}


Point::Point(double x, double y) : xCoord(x), yCoord(y) {
}


double Point::getX() const {
  return xCoord;
}


double Point::getY() const {
  return yCoord;
}


std::string Point::toString() const {
  std::ostringstream resultStream;
  resultStream << "(x: " << xCoord << ", y: " << yCoord << ")";
  return resultStream.str();
}


GridCell::GridCell() : col(0), row(0) {
}


GridCell::GridCell(int col, int row) : col(col), row(row) {
}


int GridCell::getRow() const {
  return row;
}


void GridCell::setRow(int row) {
  GridCell::row = row;
}


int GridCell::getCol() const {
  return col;
}


void GridCell::setCol(int col) {
  GridCell::col = col;
}


bool GridCell::equals(const GridCell cell) const {
  return (col == cell.getCol() && row == cell.getRow());
}


bool GridCell::operator==(const GridCell &cell) {
  return (col == cell.getCol() && row == cell.getRow());
}


bool GridCell::operator!=(const GridCell &cell) {
  return (col != cell.getCol() || row != cell.getRow());
}


std::string GridCell::toString() const {
  std::ostringstream resultStream;
  resultStream << "(c: " << col << ", r: " << row << ")";
  return resultStream.str();
}


GPS::GPS() : longitude(0), latitude(0) {
}


GPS::GPS(double longi, double lat) : longitude(longi), latitude(lat) {
}


double GPS::getLong() const {
  return longitude;
}


void GPS::setLong(const double longi) {
  longitude = longi;
}


double GPS::getLat() const {
  return latitude;
}


void GPS::setLat(const double lat) {
  latitude = lat;
}


std::string GPS::toString() const {
  std::ostringstream resultStream;
  resultStream << "(lon: " << longitude << ", lat: " << latitude << ")";
  return resultStream.str();
}
