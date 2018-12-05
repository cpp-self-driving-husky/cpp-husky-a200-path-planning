#include <utility>

/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WavefrontNavigator.cpp
 */

#include <array>
#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#include <unordered_map>
#include <limits>
#include "WavefrontNavigator.h"
#include "Point.h"
#include "GridCell.h"
#include "OccGrid.h"

typedef std::list<GridCell>::iterator list_it;

WavefrontNavigator::WavefrontNavigator() = default;

WavefrontNavigator::WavefrontNavigator(const std::string filename) {
    mapfilename = filename;
    initializeNavigator();
    loadDestinations("../dest_coordinates.txt");
    loadTestXY("../test_xy.txt");
}

WavefrontNavigator::~WavefrontNavigator() = default;

void WavefrontNavigator::initializeNavigator() {
    gridMap.resetGrid();
    gridMap.inputGrid(mapfilename, 5.0);
    gridMap.outputGrid("../debugOutput/1-scaled input map.pnm");
    gridMap.growGrid(0.20);
    gridMap.outputGrid("../debugOutput/2-grow.pnm");

    debugGrid.resetGrid();
    debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);

    wayCells.clear();
}

std::unordered_map<std::string, Point> WavefrontNavigator::getDestinations() {
    return destinations;
}

std::unordered_map<std::string, Point> WavefrontNavigator::getTestXY() {
    return testXY;
}

std::list<GridCell> WavefrontNavigator::getWayCells() {
    return wayCells;
}

std::list<GridCell> WavefrontNavigator::outputPath() {
    GridCell prev, curr;
    std::list<GridCell> resultPath;
    std::list<GridCell>::iterator iter;
    iter = wayCells.begin();

    do {
        prev = *iter;
        iter++;
        curr = *iter;
        std::list<GridCell> lineSegment = drawLine(prev, curr);
        resultPath.splice(resultPath.end(), lineSegment);
        printCells(resultPath);
    } while (iter != wayCells.end());

    return resultPath;
}

bool WavefrontNavigator::planPath(Point start, Point goal, const std::string waveType) {
    initializeNavigator();

    GridCell startCell = OccGrid::pointToCell(start);
    GridCell goalCell = OccGrid::pointToCell(goal);

    GridCell waveResult;
    bool pathFound{false};

    gridMap.normCell(startCell);
    gridMap.normCell(goalCell);

    if ((gridMap.get(goalCell) == 1) || (gridMap.get(startCell) == 1)) {
        std::cout << "   +*%$   Goal and Start locations must both be unoccupied.";
        return pathFound;
    }

    if (waveType == "Basic") {
        waveResult = gridMap.propWavesBasic(goalCell, startCell);
    } else if (waveType == "OFWF") {
        waveResult = gridMap.propOFWF(goalCell, startCell);
    }

    if (!waveResult.equals(goalCell)) {
        calcWayCells(waveResult, goalCell);
        smoothPath();
        std::cout << "\nSmoothed path:";
        printCells(wayCells);
        markCells(wayCells);
//        markCells(outputPath());
        debugGrid.outputGrid("../debugOutput/4-smoothpath.pnm");
        debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);
        pathFound = true;
        return pathFound;
    } else {
        std::cout << "      $ ERROR: cannot find a path to that location.\n" << std::endl;
        return pathFound;
    }
}

bool WavefrontNavigator::planPath(GridCell start, GridCell goal, const std::string waveType) {
    initializeNavigator();

    GridCell waveResult;
    bool pathFound{false};

    gridMap.normCell(start);
    gridMap.normCell(goal);

    if (waveType == "Basic") {
        waveResult = gridMap.propWavesBasic(goal, start);
    } else if (waveType == "OFWF") {
        waveResult = gridMap.propOFWF(goal, start);
    }

    if (!waveResult.equals(goal)) {
        calcWayCells(waveResult, goal);
        smoothPath();
        std::cout << "\nSmoothed path:";
        printCells(wayCells);
        markCells(wayCells);
//        markCells(outputPath());
        debugGrid.outputGrid("../debugOutput/4-smoothpath.pnm");
        debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);
        pathFound = true;
        return pathFound;
    } else {
        std::cout << "      $ ERROR: cannot find a path to that location.\n" << std::endl;
        return pathFound;
    }
}

void WavefrontNavigator::printCells(std::list<GridCell> cells) {
    std::cout << "\n(col, row)\n";
    for (const auto &cell : cells) {
        cell.printCell();
    }

}

void WavefrontNavigator::markCells(std::list<GridCell> cells) {
    for (const auto &cell : cells) {
        debugGrid.set(cell, 1);
    }

}

void WavefrontNavigator::loadDestinations(std::string filename) {
    std::ifstream ifs;
    std::string name;
    double latitude, longitude;
    Point destination;

    ifs.open(filename.c_str(), std::ifstream::in);
    if (ifs) {
        while (ifs >> name) {
            ifs >> latitude >> longitude;
            destination = Point(OccGrid::longitudeToX(longitude), OccGrid::latitudeToY(latitude));
            destinations.insert({name, destination});
        }
        ifs.close();
    }
}

void WavefrontNavigator::loadTestXY(std::string filename) {
    std::ifstream ifs;
    std::string name;
    double x, y;
    Point destination;

    ifs.open(filename.c_str(), std::ifstream::in);
    if (ifs) {
        while (ifs >> name) {
            ifs >> x >> y;
            destination = Point(x, y);
            testXY.insert({name, destination});
        }
        ifs.close();
    }
}

void WavefrontNavigator::calcWayCells(const GridCell &start, const GridCell &goal) {
    GridCell nextCell;
    for (GridCell currCell = start; !currCell.equals(goal); currCell = nextCell) {
        wayCells.emplace_back(currCell);
        debugGrid.set(currCell, 1);
        nextCell = findNextCell(currCell);
    }
    wayCells.emplace_back(goal);
    std::cout << "\n   +*%$ Found initial path to goal location.\n" << std::endl;
    debugGrid.outputGrid("../debugOutput/3-initialPath.pnm");
    debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);
}

GridCell WavefrontNavigator::findNextCell(const GridCell &currCell) {
    double currValue = gridMap.get(currCell);

    // currCell is the goal cell or is part of an obstacle
    if (currValue < 3) {
        return currCell;
    }

    // find the neighboring cell with the minimum value
    int minRow, maxRow, minCol, maxCol;
    minRow = std::max(currCell.getRow() - 1, 0);
    maxRow = std::min(currCell.getRow() + 1, gridMap.getGridHeight() - 1);
    minCol = std::max(currCell.getCol() - 1, 0);
    maxCol = std::min(currCell.getCol() + 1, gridMap.getGridWidth() - 1);
    double tempMin = currValue;
    GridCell tempMinCell;
    int nRow, nCol;
    for (nRow = minRow; nRow <= maxRow; ++nRow) {
        for (nCol = minCol; nCol <= maxCol; ++nCol) {
            currValue = gridMap.get(nRow, nCol);
            if (currValue < tempMin &&currValue > 1) {
                tempMin = currValue;
                tempMinCell = GridCell(nRow, nCol);
            }
        }
    }
    if (tempMinCell.equals(currCell)) {
        std::cout << "      $ ERROR: no neighboring cell has a lower value than current cell." << std::endl;
    }
    return tempMinCell;
}

void WavefrontNavigator::smoothPath() {
    int editCount{1};
    int n0, n1;
    while (editCount > 0) {
        n0 = static_cast<int>(wayCells.size());
        editCount = smoothPathHelper();
//        editCount += smoothPathHelperReverse();
        n1 = static_cast<int>(wayCells.size());
        std::cout << "    *%$ smoothPath: " << n0 << " -> " << n1 << " waypoints. "
                  << editCount << " trimmed." << std::endl;
    }
}

int WavefrontNavigator::smoothPathHelper() {
    if (wayCells.size() < 3) {
        return 0;
    }

    int modificationCount{0};
    std::list<GridCell>::iterator it0, it1, it2;
    it0 = it1 = it2 = wayCells.begin();
    it1++;
    it2++;
    it2++;
    while (it2 != wayCells.end()) {
        if (isInLine(*it0, *it2)) {
            it1 = wayCells.erase(it1);
            modificationCount++;
            it2++;
        } else {
            it0++;
            it1++;
            it2++;
        }
    }
    return modificationCount;
}

int WavefrontNavigator::smoothPathHelperReverse() {
    if (wayCells.size() < 3) {
        return 0;
    }

    int modificationCount{0};
    std::list<GridCell>::iterator it0, it1, it2;
    it0 = it1 = it2 = wayCells.end();
    it0--;
    it1--;
    it1--;
    it2--;
    it2--;
    it2--;
    while (it2 != wayCells.begin()) {
        if (isInLine(*it0, *it2)) {
            it1 = wayCells.erase(it1);
            modificationCount++;
            it1--;
            it2--;
        } else {
            it0--;
            it1--;
            it2--;
        }
    }
    return modificationCount;
}

/**
 * Checks to see if there is a clear line of sight between c0 and c1.
 * If a clear line exists, then any wayCells between c0 and c1 in the wayCells
 * list can be removed.
 *
 * @param c0
 * @param c1
 * @return true if there is a clear line of sight between c0 and c1
 */
bool WavefrontNavigator::isInLine(const GridCell &c0, const GridCell &c1) {
    // find the cells that lie on the line between c0 and c1.
    std::list<GridCell> lineSegment = drawLine(c0, c1);

    // check if any cell in the list is occupied by an obstacle.
    for (const auto &gc : lineSegment) {
        if (gridMap.get(gc) == 1) {
            return false;
        }
    }
    return true;
}

/**
 * Makes a list of the GridCells that are traversed while making a straight
 * line between two GridCells. Uses Bresenham's algorithm.
 *
 * @param c0 source GridCell
 * @param c1 destination GridCell
 * @return std::list<GridCell> list of GridCells that approximate a straight line between c0 and c1
 */
std::list<GridCell> WavefrontNavigator::drawLine(const GridCell &c0, const GridCell &c1) {
    int row0{c0.getRow()}, row1{c1.getRow()}, col0{c0.getCol()}, col1{c1.getCol()};
    int dRow{abs(row1 - row0)}, dCol{abs(col1 - col0)};

    int col_increment{0};
    if (col1 > col0) {
        col_increment = 1;
    } else if (col1 < col0) {
        col_increment = -1;
    }

    int row_increment{0};
    if (row1 > row0) {
        row_increment = 1;
    } else if (row1 < row0) {
        row_increment = -1;
    }
    
    int error = dCol - dRow;
    dCol *= 2;
    dRow *= 2;

    std::list<GridCell> lineOfCells;
    int currRow{row0}, currCol{col0};

    for (int cellCount = 1 + dRow + dCol; cellCount > 0; --cellCount) {
        lineOfCells.emplace_back(GridCell(currRow, currCol));

        // error value basically tells us if the current cell lies above or below the actual line.
        // We use this info to find the next cell that is closest to the line.
        if (error > 0) {
            currCol += col_increment;
            error -= dRow;
        } else if (error < 0) {
            currRow += row_increment;
            error += dCol;
        } else if (error == 0) {
            currCol += col_increment;
            error -= dRow;
            currRow += row_increment;
            error += dCol;
            cellCount--;
        }
    }
    return lineOfCells;
}

OccGrid* WavefrontNavigator::getGridMap() {
    return &gridMap;
}

OccGrid* WavefrontNavigator::getDebugGrid() {
    return &debugGrid;
}


/**
 * Main program driver
 *
 * @param myNav
 * @param startTitle
 * @param goalTitle
 */

void findPath(WavefrontNavigator &myNav, const std::string &waveOption, const std::string &startTitle, const std::string &goalTitle) {
    std::cout << "\n--+*%$ Finding Path between " << startTitle << " and " << goalTitle << std::endl;
    Point start, goal;
    try {
        start = myNav.getDestinations().at(startTitle);
        goal = myNav.getDestinations().at(goalTitle);
    } catch (const std::out_of_range &oor) {
        start = myNav.getTestXY().at(startTitle);
        goal = myNav.getTestXY().at(goalTitle);
    }
    std::cout << "       " << startTitle << ": " << start.getX() << ", " << start.getY() << std::endl;
    std::cout << "       " << goalTitle << ": " << goal.getX() << ", " << goal.getY() << std::endl;
    if (myNav.planPath(start, goal, waveOption)) {
        std::cout << "SUCCESS\n";
    }
}

std::list<GridCell> findAnyPath(WavefrontNavigator &myNav, const std::string &waveOption, Coord startC, Coord goalC) {
    GridCell start = OccGrid::coordToCell(startC);
    GridCell goal = OccGrid::coordToCell(goalC);
    myNav.planPath(start, goal, "OFWF");
    return myNav.getWayCells();
}

int main(int argc, char *argv[]) {
    WavefrontNavigator myNav("../bitmaps/2dmap-01.pnm");
//    findPath(myNav, "OFWF", "dest01", "dest02");
//    findPath(myNav, "OFWF", "dest01", "dest03");
//    findPath(myNav, "OFWF", "dest01", "dest04");
    findPath(myNav, "OFWF", "dest06", "dest05");
//    findPath(myNav, "OFWF", "dest06", "dest09");
//    findPath(myNav, "OFWF", "dest06", "dest10");
//    findPath(myNav, "OFWF", "dest01", "dest10");
//    findPath(myNav, "Basic", "dest01", "dest10");
    Coord coord1 = Coord(-117.823690, 34.0578);
    Coord coord2 = Coord(-117.8228, 34.0588);

    findAnyPath(myNav, "OFWF", coord1, coord2);


}

