/**
 * project: CATE
 *    team: Behavioral (Path Planning)
 *  author: Allen Kim
 *    file: Coordinates.cpp
 */


#include "../../include/pathplanning/Coordinates.h"

namespace pathplanner {


Point::Point() : xCoord_(0), yCoord_(0) {
}


Point::Point(double x, double y) : xCoord_(x), yCoord_(y) {
}


double Point::x() const {
  return xCoord_;
}


double Point::y() const {
  return yCoord_;
}


std::string Point::toString() const {
  std::ostringstream resultStream;
  resultStream << "(x: " << xCoord_ << ", y: " << yCoord_ << ")";
  return resultStream.str();
}


GridCell::GridCell() : col_(0), row_(0) {
}


GridCell::GridCell(int col, int row) : col_(col), row_(row) {
}


int GridCell::getRow() const {
  return row_;
}


void GridCell::setRow(int newRow) {
  GridCell::row_ = newRow;
}


int GridCell::getCol() const {
  return col_;
}


void GridCell::setCol(int newCol) {
  GridCell::col_ = newCol;
}


bool GridCell::equals(const GridCell &cell) const {
  return (col_ == cell.getCol() && row_ == cell.getRow());
}


std::string GridCell::toString() const {
  std::ostringstream resultStream;
  resultStream << "(c: " << col_ << ", r: " << row_ << ")";
  return resultStream.str();
}


bool operator==(const GridCell &cell1, const GridCell &cell2) {
  return (cell1.getCol() == cell2.getCol() && cell1.getRow() == cell2.getRow());
}


bool operator!=(const GridCell &cell1, const GridCell &cell2) {
  return (cell1.getCol() != cell2.getCol() || cell1.getRow() != cell2.getRow());
}


GPS::GPS() : longitude_(0), latitude_(0) {
}


GPS::GPS(double longi, double lat) : longitude_(longi), latitude_(lat) {
}


double GPS::getLongitude() const {
  return longitude_;
}


void GPS::setLongitude(double newLongitude) {
  longitude_ = newLongitude;
}


double GPS::getLatitude() const {
  return latitude_;
}


void GPS::setLatitude(double newLatitude) {
  latitude_ = newLatitude;
}


std::string GPS::toString() const {
  std::ostringstream resultStream;
  resultStream << "(lon: " << longitude_ << ", lat: " << latitude_ << ")";
  return resultStream.str();
}

}
