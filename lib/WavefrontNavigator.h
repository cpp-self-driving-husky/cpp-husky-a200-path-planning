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


class WavefrontNavigator {

public:
    WavefrontNavigator();
    explicit WavefrontNavigator(std::string mapfilename);
    ~WavefrontNavigator();
    bool planPath(Point start, Point goal);
    std::list<GridCell> getWayCells();
    std::list<GridCell> outputPath();
    void printCells(std::list<GridCell> cells);
    void markCells(std::list<GridCell> cells);
    void loadDestinations(std::string filename);
    std::unordered_map<std::string, Point> destinations;
    OccGrid gridMap;

protected:
    char inputLine1[80];

private:
    std::list<GridCell> wayCells;
    void calcWayCells(GridCell start, GridCell goal);
    GridCell findNextCell(GridCell currCell);
    void smoothPath();
    int smoothPathHelper();
    bool isInLine(GridCell c1, GridCell c2);
    std::list<GridCell> drawLine(GridCell c0, GridCell c1);
    OccGrid debugGrid;
};


#endif //WAVEFRONT_WAVEFRONTNAVIGATOR_H
