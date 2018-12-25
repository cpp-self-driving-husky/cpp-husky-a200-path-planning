/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: Coordinates.h
 */

/**
 * This file contains 3 different classes: Point, GridCell, GPS. These are the 3 ways to reference a map location.
 *
 *    GPS: (longitude, latitude) GPS coordinates as decimal degrees.
 *
 *    Point: (x, y) coordinates as doubles. Origin is at top left of the map, with positive x direction Pointing to the right,
 *        and positive y direction Pointing down. x and y are in units of 1 meter.
 *
 *    GridCell: (col, row) as long indices of 2D array that represents a lower-resolution grid. Same orientation as the (x, y)
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


/**
 * A Point stores the (x, y) coordinates of a location.
 */
class Point {
public:
  Point();
  Point(double x, double y);
  virtual ~Point() = default;
  double getX() const;
  double getY() const;
  std::string toString() const;

protected:

private:
  double xCoord;
  double yCoord;
};


/**
 * A GridCell stores the occupancy grid coordinates of a cell as a (col, row) pair.
 */
class GridCell {
public:
  GridCell();
  GridCell(long col, long row);
  virtual ~GridCell() = default;
  long getRow() const;
  void setRow(long row);
  long getCol() const;
  void setCol(long col);

  bool equals(GridCell cell) const;
  std::string toString() const;

protected:

private:
  long col;
  long row;
};

bool operator==(const GridCell &cell1, const GridCell &cell2);
bool operator!=(const GridCell &cell1, const GridCell &cell2);


/**
 * A Coord stores the latitude and longitude of a location.
 */
class GPS {
public:
  GPS();
  GPS(double longitude, double latitude);
  virtual ~GPS() = default;
  double getLong() const;
  void setLong(double longi);
  double getLat() const;
  void setLat(double lat);
  std::string toString() const;

protected:

private:
  double longitude;
  double latitude;
};

#endif //WAVEFRONT_COORDINATES_H
