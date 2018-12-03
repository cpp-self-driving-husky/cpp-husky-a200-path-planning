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


WavefrontNavigator::WavefrontNavigator() = default;

WavefrontNavigator::WavefrontNavigator(std::string mapfilename) {
    gridMap.inputGrid(std::move(mapfilename), SCALE_MAP);
    gridMap.outputGrid("../debugOutput/1-scaled input map.pnm");

    loadDestinations("../dest_coordinates.txt");
    loadTestXY("../test_xy.txt");
}

WavefrontNavigator::~WavefrontNavigator() = default;

bool WavefrontNavigator::planPath(Point start, Point goal, std::string waveType) {
    gridMap.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);

    // expand the boundaries of the obstacles so we can represent the Husky as a point
    gridMap.growGrid(0.30);
    gridMap.outputGrid("../debugOutput/2-grow.pnm");

    // debugGrid is used to visually mark the results of the various steps of the program
    debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);

    GridCell startCell = OccGrid::pointToCell(start);
    GridCell goalCell = OccGrid::pointToCell(goal);

    GridCell waveResult;
    bool pathFound{false};

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

void WavefrontNavigator::printCells(std::list<GridCell> cells) {
    std::cout << "\n(col, row)\n";
    for (auto cell : cells) {
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

void WavefrontNavigator::calcWayCells(GridCell start, GridCell goal) {
    GridCell nextCell;
    for (GridCell currCell = start; !currCell.equals(goal); currCell = nextCell) {
        wayCells.push_back(currCell);
        debugGrid.set(currCell, 1);
        nextCell = findNextCell(currCell);
    }
    wayCells.push_back(goal);
    std::cout << "\n   +*%$ Found initial path to goal location.\n" << std::endl;
    debugGrid.outputGrid("../debugOutput/3-initialPath.pnm");
    debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);
}

GridCell WavefrontNavigator::findNextCell(GridCell currCell) {
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
            if (currValue < tempMin && currValue > 1) {
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

/**
 * Checks to see if there is a clear line of sight between c0 and c1.
 * If a clear line exists, then any wayCells between c0 and c1 in the wayCells
 * list can be removed.
 *
 * @param c0
 * @param c1
 * @return true if there is a clear line of sight between c0 and c1
 */
bool WavefrontNavigator::isInLine(GridCell c0, GridCell c1) {
    // find the cells that lie on the line between c0 and c1.
    std::list<GridCell> lineSegment = drawLine(c0, c1);

    // check if any cell in the list is occupied by an obstacle.
    for (const auto &gc : lineSegment) {
        if (gridMap.get(gc) == 1.0) {
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
std::list<GridCell> WavefrontNavigator::drawLine(GridCell c0, GridCell c1) {
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
        lineOfCells.emplace_back(currRow, currCol);

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

std::unordered_map<std::string, Point> WavefrontNavigator::getDestinations() {
    return destinations;
}

std::unordered_map<std::string, Point> WavefrontNavigator::getTestXY() {
    return testXY;
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

int main(int argc, char *argv[]) {
    WavefrontNavigator myNav("../bitmaps/2dmap-01.pnm");
    findPath(myNav, "OFWF", "dest01", "dest02");
    findPath(myNav, "OFWF", "dest01", "dest03");
    findPath(myNav, "OFWF", "dest01", "dest04");
    findPath(myNav, "OFWF", "dest06", "dest01");
    findPath(myNav, "OFWF", "dest06", "dest09");
    findPath(myNav, "OFWF", "dest06", "dest10");
    findPath(myNav, "OFWF", "dest01", "dest10");
//    findPath(myNav, "Basic", "dest01", "dest10");

}

