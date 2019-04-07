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

#include "Coordinates.h"

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
    virtual ~OccGrid();
    void inputGrid(std::string filename, double mapScale);
    void convertToDebugGrid();
    void outputGrid(std::string filename);
    void outputDebugGrid(std::string filename);
    void resetGrid();
    long get(long col, long row);
    long get(GridCell cell);
    void set(GridCell cell, long value);
    void growGrid(double radius);
    std::pair<GridCell, long> propWavesBasic(GridCell &goal, GridCell &start, long orthoDist, long diagDist);
    std::pair<GridCell, long> propOFWF(GridCell &goal, GridCell &start, long orthoDist, long diagDist);
    std::vector<GridCell> getNeighbors(const GridCell &cell, int layers);
    void normCell(GridCell& cell);

protected:
    long maxVal;
    char inputLine1[80];
    std::array<std::array<long, GRID_WIDTH>, GRID_HEIGHT> grid;

private:
    void setWeight(GridCell &cell, long orthoDist, long diagDist);
    bool isNear(const GridCell& c1, const GridCell& c2);
    GridCell findClosestFreeCell(GridCell& cell);
    std::pair<long, GridCell> findFree(GridCell &cell, std::string direction);
    void fitInGrid(GridCell& cell);
};

GridCell PointToCell(const Point &pt);
GPS PointToGPS(const Point &pt);
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
