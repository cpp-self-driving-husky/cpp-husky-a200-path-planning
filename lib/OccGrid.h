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
#include "Point.h"
#include "GridCell.h"
#include "Coord.h"

#define SCALE_MAP 5.0
#define WIDTH_METERS 186.23
#define HEIGHT_METERS 186.23
#define GRID_HEIGHT 373
#define GRID_WIDTH 373
#define MIN_LAT 34.057649
#define MAX_LAT 34.059320
#define MIN_LONG -117.824690
#define MAX_LONG -117.822673


/**
 * Occupancy Grid class. Contains methods for converting GPS coordinates into (x, y) cartesian
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
    void outputGrid(std::string filename);
    void resetGrid();
    int get(int row, int col);
    int get(GridCell cell);
    void set(GridCell cell, int value);
    static GridCell pointToCell(Point pt);
    static Point cellToPoint(GridCell cell);
    static GridCell coordToCell(Coord coord);
    static Point coordToPoint(const Coord &coord);

    void growGrid(double radius);
    int getGridWidth() const;
    int getGridHeight() const;
    GridCell propWavesBasic(GridCell goal, GridCell start);
    GridCell propOFWF(GridCell goal, GridCell start);

    static int xToCol(double x);
    static int yToRow(double y);
    static double colToX(int n);
    static double rowToY(int m);
    static double xToLongitude(double x);
    static double yToLatitude(double y);
    static double longitudeToX(double longitude);
    static double latitudeToY(double latitude);
    void normCell(GridCell& cell);


protected:
    int maxVal;
    char inputLine1[80];
    std::array<std::array<int, GRID_WIDTH>, GRID_HEIGHT> grid {{{0}}};

private:
    void setWeightMWF(GridCell& cell);
    bool isNear(const GridCell& c1, const GridCell& c2);
    GridCell findClosestFreeCell(GridCell &cell);
    void fitInGrid(GridCell &cell);

};


#endif //WAVEFRONT_OCCGRID_H
