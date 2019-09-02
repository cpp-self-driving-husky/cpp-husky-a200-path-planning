/**
 * project: CATE
 *    team: Behavioral (Path Planning)
 *  author: Allen Kim
 *    file: Coordinates.h
 */

/**
 * This file contains 3 different classes: Point, GridCell, GPS. These are the 3 ways to reference a map location.
 *
 *    GPS: (longitude, latitude) GPS coordinates as decimal degrees.
 *
 *    Point: (x, y) coordinates as doubles. Origin is at top left of the map, with positive x direction Pointing to the right,
 *        and positive y direction Pointing down. x and y are in units of 1 meter.
 *
 *    GridCell: (col, row) as int indices of 2D array that represents a lower-resolution grid. Same orientation as the (x, y)
 *        grid used for Points. In this implementation, a grid cell is 0.2m x 0.2m.
 *
 * Note the order in which the coordinate pairs are listed. They all reference the horizontal component first,
 * then the vertical. This is the reverse of 2D array indices -- usually (row, col).
 *
 * Functions that perform conversions between these location representations are in OccGrid.h because the conversions
 * are dependent on the Occupancy Grid dimensions.
 */

#ifndef WAVEFRONT_COORDINATES_H
#define WAVEFRONT_COORDINATES_H

#include <iostream>
#include <sstream>
#include <string>

namespace pathplanner {

/**
 * A Point stores the (x, y) coordinates of a location.
 */
class Point {
public:
  Point();
  Point(double x, double y);
  virtual ~Point() = default;
  double x() const;
  double y() const;
  std::string toString() const;

private:
  double xCoord_;
  double yCoord_;
};

/**
 * A GridCell stores the occupancy grid coordinates of a location as a (col, row) pair.
 */
class GridCell {
public:
  GridCell();
  GridCell(int col, int row);
  virtual ~GridCell() = default;
  int getRow() const;
  void setRow(int newRow);
  int getCol() const;
  void setCol(int newCol);
  bool equals(const GridCell &cell) const;
  std::string toString() const;

private:
  int col_;
  int row_;
};

bool operator==(const GridCell &cell1, const GridCell &cell2);
bool operator!=(const GridCell &cell1, const GridCell &cell2);


/**
 * A GPS stores a location as a (longitude, latitude) pair.
 */
class GPS {
public:
  GPS();
  GPS(double longitude, double latitude);
  virtual ~GPS() = default;
  double getLongitude() const;
  void setLongitude(double newLongitude);
  double getLatitude() const;
  void setLatitude(double newLatitude);
  std::string toString() const;

private:
  double longitude_;
  double latitude_;
};

}
#endif //WAVEFRONT_COORDINATES_H
