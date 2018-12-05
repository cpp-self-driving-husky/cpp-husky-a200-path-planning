/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WavefrontNavigator.h
 */


#ifndef WAVEFRONT_WAVEFRONTNAVIGATOR_H
#define WAVEFRONT_WAVEFRONTNAVIGATOR_H

#include <string>
#include <unordered_map>
#include <list>
#include "OccGrid.h"
#include "OccGrid.cpp"
#include "Coord.h"


class WavefrontNavigator {

public:
    WavefrontNavigator();
    explicit WavefrontNavigator(std::string filename);
    ~WavefrontNavigator();
    std::unordered_map<std::string, Point> getDestinations();
    std::unordered_map<std::string, Point> getTestXY();
    OccGrid* getGridMap();
    OccGrid* getDebugGrid();
    std::list<GridCell> getWayCells();
    std::list<GridCell> outputPath();
    bool planPath(Point start, Point goal, std::string waveType);
    bool planPath(GridCell start, GridCell goal, std::string waveType);
    void printCells(std::list<GridCell> cells);
    void markCells(std::list<GridCell> cells);
    void loadDestinations(std::string filename);
    void loadTestXY(std::string filename);

protected:
    char inputLine1[80];

private:
    std::list<GridCell> wayCells;
    std::unordered_map<std::string, Point> destinations;
    std::unordered_map<std::string, Point> testXY;
    OccGrid gridMap;
    OccGrid debugGrid;
    std::string mapfilename;
    void initializeNavigator();
    void calcWayCells(const GridCell &start, const GridCell &goal);
    GridCell findNextCell(const GridCell &currCell);
    void smoothPath();
    int smoothPathHelper();
    int smoothPathHelperReverse();
    bool isInLine(const GridCell &c1, const GridCell &c2);
    std::list<GridCell> drawLine(const GridCell &c0, const GridCell &c1);
};


#endif //WAVEFRONT_WAVEFRONTNAVIGATOR_H
