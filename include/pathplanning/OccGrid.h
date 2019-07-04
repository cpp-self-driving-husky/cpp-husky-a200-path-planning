/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: OccGrid.h
 */

#ifndef WAVEFRONT_OCCGRID_H
#define WAVEFRONT_OCCGRID_H

#include <string>
#include <array>
#include <list>
#include <queue>
#include <chrono>
#include <unordered_map>
#include <vector>

#include "../../include/pathplanning/Coordinates.h"
#include "../../include/pathplanning/OccGrid.h"

const double SCALE_MAP = 5.0;
const double WIDTH_METERS = 186.23;
const double HEIGHT_METERS = 186.23;
const long GRID_HEIGHT = 373;
const long GRID_WIDTH = 373;
const double MIN_LAT = 34.057649;
const double MAX_LAT = 34.059320;
const double MIN_LONG = -117.824690;
const double MAX_LONG = -117.822673;

/**
 * Occupancy Grid class. Contains methods for converting GPS coordinates longo (x, y) cartesian
 * coordinates and grid cell coordinates. Also includes methods for manipulating occupancy grids
 * (i.e., growing obstacles, propagating waves).
 *
 * (Note: this class is designed to work only with the "2dmap-01.pnm" bitmap.)
 */

class OccGrid {

public:
  OccGrid();
  OccGrid(const std::string &filename, double mapScale);
  virtual ~OccGrid();
  long get(long col, long row);
  long get(const GridCell &cell);
  void set(const GridCell &cell, long value);
  void convertToDebugGrid();
  void outputGrid(const std::string &filename);
  void outputDebugGrid(const std::string &filename);
  void outputWaves(const std::string &filename, const std::string &color);
  OccGrid growGrid(double radius);
  std::pair<GridCell, long> propWavesBasic(GridCell &goal, GridCell &start, long orthoDist, long diagDist);
  std::pair<GridCell, long> propOFWF(GridCell &goal, GridCell &start, long orthoDist, long diagDist);
  static std::vector<GridCell> getNeighborhood(const GridCell &cell, long layers);
  void normCell(GridCell &cell);
  bool isInLine(const GridCell &c1, const GridCell &c2);
  static bool isNear(const GridCell &c1, const GridCell &c2);
  static std::vector<GridCell> drawLine(const GridCell &cell0, const GridCell &cell1);
  static double euclideanDist(const GridCell &c0, const GridCell &c1);
  void markCell(const GridCell& cell, long value);
  void markCells(const std::list<GridCell>& cells, long value);
  void markCells(const std::vector<GridCell> &cells, long value);

private:
  std::vector<std::vector<long> > grid;
  void setWeight(GridCell &cell, long orthoDist, long diagDist);
  GridCell findClosestFreeCell(GridCell &cell);
  std::pair<long, GridCell> findFree(GridCell &cell, const std::string &direction);
  void fitInGrid(GridCell &cell);
  long findMaxDistance();
  static std::vector<GridCell> drawLineLow(const long c0, const long r0, const long c1, const long r1);
  static std::vector<GridCell> drawLineHigh(const long c0, const long r0, const long c1, const long r1);
};

GridCell pointToCell(const Point &pt);
GPS pointToGPS(const Point &pt);
Point cellToPoint(const GridCell &cell);
GPS cellToGPS(const GridCell &cell);
GridCell gpsToCell(const GPS &gps);
Point gpsToPoint(const GPS &gps);
long xToCol(double x);
long yToRow(double y);
double colToX(long n);
double rowToY(long m);
double xToLongitude(double x);
double yToLatitude(double y);
double longitudeToX(double longitude);
double latitudeToY(double latitude);

#endif //WAVEFRONT_OCCGRID_H
