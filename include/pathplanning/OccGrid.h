/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: OccGrid.h
 */

#ifndef WAVEFRONT_OCCGRID_H
#define WAVEFRONT_OCCGRID_H

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "Coordinates.h"

namespace pathplanner {

const double SCALE_MAP = 4.0;
const double WIDTH_METERS = 186.23;
const double HEIGHT_METERS = 186.23;
const int GRID_HEIGHT = 466;
const int GRID_WIDTH = 466;
const double MIN_LAT = 34.057649;
const double MAX_LAT = 34.059320;
const double MIN_LONG = -117.824690;
const double MAX_LONG = -117.822673;

/**
 * Occupancy Grid class. Includes methods for manipulating occupancy grids
 * (i.e., growing obstacles, propagating waves).
 *
 * This file also contains conversion functions for all coordinate systems.
 *
 * (Note: this class is designed to work only with the "2dmap-01.pnm" bitmap.)
 */

class OccGrid {

public:
  OccGrid();
  OccGrid(const std::string &filename, double mapScale);
  virtual ~OccGrid();
  int get(int col, int row) const;
  int get(const GridCell &cell) const;
  void set(const GridCell &cell, int value);
  void outputGrid(const std::string &filename);
  void outputGrid2(const std::string &filename);
  OccGrid growGrid(double radius);
  std::pair<GridCell, int> propWavesBasic(GridCell &goal, GridCell &start, int orthoDist, int diagDist);
  std::pair<GridCell, int> propOFWF(GridCell &goal, GridCell &start, int orthoDist, int diagDist);
  void getNeighborhood(const GridCell &cell,
                       int layers,
                       std::vector<GridCell> &neighborhood);
  void normCell(GridCell &cell);
  bool isInLine(const GridCell &c1, const GridCell &c2);
  static bool isNear(const GridCell &c1, const GridCell &c2);
  static std::vector<GridCell> drawLine(const GridCell &cell0, const GridCell &cell1);
  static double euclideanDist(const GridCell &c0, const GridCell &c1);
  int findMaxDistance() const;

private:
  int gridWidth_;
  int gridHeight_;
  std::vector<std::vector<int> > grid_;
  void setWeight(GridCell &cell, int orthoDist, int diagDist);
  GridCell findClosestFreeCell(GridCell &cell);
  std::pair<int, GridCell> findFree(GridCell &cell, const std::string &direction);
  void fitInGrid(GridCell &cell);
  static std::vector<GridCell> drawLineLow(int c0, int r0, int c1, int r1);
  static std::vector<GridCell> drawLineHigh(int c0, int r0, int c1, int r1);
};

GridCell pointToCell(const Point &pt);
GPS pointToGPS(const Point &pt);
Point cellToPoint(const GridCell &cell);
GPS cellToGPS(const GridCell &cell);
GridCell gpsToCell(const GPS &gps);
Point gpsToPoint(const GPS &gps);
int xToCol(double x);
int yToRow(double y);
double colToX(int n);
double rowToY(int m);
double xToLongitude(double x);
double yToLatitude(double y);
double longitudeToX(double longitude);
double latitudeToY(double latitude);

}
#endif //WAVEFRONT_OCCGRID_H
