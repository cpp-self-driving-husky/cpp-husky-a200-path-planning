/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: OccGrid.cpp
 */

#include <array>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <queue>
#include <cstring>
#include <utility>
#include "OccGrid.h"

class CompareGreater {
public:
    bool operator()(std::pair<double, GridCell> &p1, std::pair<double, GridCell> &p2) {
        return p1.first > p2.first;
    }
    bool operator()(std::pair<int, GridCell> &p1, std::pair<int, GridCell> &p2) {
        return p1.first > p2.first;
    }
};

OccGrid::OccGrid() = default;

OccGrid::~OccGrid() = default;

void OccGrid::inputGrid(const std::string filename, const double mapScale = SCALE_MAP) {
    resetGrid();
    int inRow, inCol;
    int inputWidth, inputHeight;
    char nextChar;

    std::ifstream ifs;
    ifs.open(filename.c_str(), std::ifstream::in);

    /* Read past first line */
    ifs.getline(inputLine1, 80);

    /* Read in inputWidth, inputHeight, maxVal */
    ifs >> inputWidth >> inputHeight >> maxVal;

    /* Read in map, fill grid */
    for (inRow = 0; inRow < inputHeight; ++inRow) {
        for (inCol = 0; inCol < inputWidth; ++inCol) {
            ifs >> nextChar;
            // char(-1) == 255, which denotes an unoccupied pixel in input image
            if (nextChar != char(-1)) {
                grid[static_cast<int>(inRow / mapScale)][static_cast<int>(inCol / mapScale)] = 1;
            }
        }
    }
    ifs.close();
}

/**
 * Outputs the OccGrid as a .pnm bitmap where occupied cells are black, open cells are white.
 * 
 * @param filename 
 */
void OccGrid::outputGrid(std::string filename) {
    std::ofstream outFile(filename);
    outFile << "P5" << std::endl;
    outFile << GRID_WIDTH << " " << GRID_HEIGHT << std::endl << maxVal << std::endl;

    int row, col;
    for (row = 0; row != GRID_HEIGHT; ++row) {
        for (col = 0; col != GRID_WIDTH; ++col) {
            if (grid[row][col] == 1) {
                outFile << char(0);
            } else {
                outFile << char(-1);
            }
        }
    }
    std::cout << "   --+* Map output to " << filename << std::endl;
    outFile.close();
}

void OccGrid::resetGrid() {
    for (auto &row : grid) {
        row.fill(0);
    }
}

int OccGrid::get(int row, int col) {
    return grid[row][col];
}

int OccGrid::get(GridCell cell) {
    int row{cell.getRow()};
    int col{cell.getCol()};
    return OccGrid::get(row, col);
}

void OccGrid::set(GridCell cell, int value) {
    int row{cell.getRow()};
    int col{cell.getCol()};
    grid[row][col] = value;
}

GridCell OccGrid::pointToCell(Point pt) {
    return GridCell(yToRow(pt.getY()), xToCol(pt.getX()));
}

Point OccGrid::cellToPoint(GridCell cell) {
    return Point(colToX(cell.getCol()), rowToY(cell.getRow()));
}

GridCell OccGrid::coordToCell(Coord coord) {
    return pointToCell(coordToPoint(coord));
}

Point OccGrid::coordToPoint(const Coord &coord) {
    double longitude, latitude, x, y;
    longitude = coord.getLong();
    latitude = coord.getLat();
    double widthLong = MAX_LONG - MIN_LONG;
    double heightLat = MAX_LAT - MIN_LAT;
    x = ((longitude - MIN_LONG) / widthLong) * WIDTH_METERS;
    y = ((MAX_LAT - latitude) / heightLat) * HEIGHT_METERS;
    return Point(x, y);
}

int OccGrid::xToCol(double x) {
    return static_cast<int>((static_cast<double>(GRID_WIDTH) / WIDTH_METERS) * x);
}

int OccGrid::yToRow(double y) {
    return static_cast<int>((static_cast<double>(GRID_HEIGHT) / HEIGHT_METERS) * y);
}

double OccGrid::colToX(int n) {
    double metersPerCol = WIDTH_METERS / static_cast<double>(GRID_WIDTH);
    return metersPerCol * n;
}

double OccGrid::rowToY(int m) {
    double metersPerRow = HEIGHT_METERS / static_cast<double>(GRID_HEIGHT);
    return metersPerRow * m;
}

double OccGrid::xToLongitude(double x) {
    double widthLong = MAX_LONG - MIN_LONG;
    double longitude = ((x / WIDTH_METERS) * widthLong) + MIN_LONG;
    return longitude;
}

double OccGrid::yToLatitude(double y) {
    double heightLat = MAX_LAT - MIN_LAT;
    double latitude = ((1 - (y / HEIGHT_METERS)) * heightLat) + MIN_LAT;
    return latitude;
}

double OccGrid::longitudeToX(double longitude) {
    return WIDTH_METERS * (longitude - MIN_LONG) / (MAX_LONG - MIN_LONG);
}

double OccGrid::latitudeToY(double latitude) {
    return HEIGHT_METERS * (MAX_LAT - latitude) / (MAX_LAT - MIN_LAT);
}

/**
 * Expands obstacle boundaries by a certain radius. This is done so that the robot
 * can be represented as a point rather than a 3 dimensional object.
 * @param radius - distance in meters to expand obstacles.
 */
void OccGrid::growGrid(double radius) {
    // calculate # of grid cells to use for radius
    int growH, growV, growSize;
    double ratioH, ratioV;
    ratioV = static_cast<double>(GRID_HEIGHT) / HEIGHT_METERS;
    ratioH = static_cast<double>(GRID_WIDTH) / WIDTH_METERS;
    growV = static_cast<int>(ceil(radius * ratioV));
    growH = static_cast<int>(ceil(radius * ratioH));
    growSize = std::max(growH, growV);

    // create a new 2d array to hold the result of grow.
    std::array<std::array<int, GRID_WIDTH>, GRID_HEIGHT> result {{{0}}};

    // loop through grid, looking for 1, then growing them on result
    int row, col, minRow, maxRow, minCol, maxCol, growRow, growCol;
    for (row = 0; row != GRID_HEIGHT; ++row) {
        for (col = 0; col != GRID_WIDTH; ++col) {
            if (grid[row][col] == 1) {
                // set growSize # of neighboring cells in each direction to 1;
                // we are using 8-way neighbors
                minRow = std::max(row - growSize, 0);
                maxRow = std::min(row + growSize, GRID_HEIGHT - 1);
                minCol = std::max(col - growSize, 0);
                maxCol = std::min(col + growSize, GRID_WIDTH - 1);
                for (growRow = minRow; growRow <= maxRow; ++growRow) {
                    for (growCol = minCol; growCol <= maxCol; ++growCol) {
                        result[growRow][growCol] = 1;
                    }
                }
            }
        }
    }

    // deep copy values from result into grid
    std::copy(&result[0][0], &result[0][0] + GRID_HEIGHT * GRID_WIDTH, &grid[0][0]);
}

int OccGrid::getGridWidth() const {
    return GRID_WIDTH;
}

int OccGrid::getGridHeight() const {
    return GRID_HEIGHT;
}

/**
 * Basic wave propagation. 8-cell, square neighborhoods with wave values differing by 1.
 *
 * @param goal
 * @param start
 * @return If path is found, return the GridCell that the search stopped on. If path is not found, return goal.
 */
GridCell OccGrid::propWavesBasic(GridCell goal, GridCell start) {
    std::cout << "\n --+*%$ Waves - 8-cell, square neighborhood" << std::endl;
    std::cout << "     %$    start cell = (c: " << start.getCol() << ", r: " << start.getRow() << ")" << std::endl;
    std::cout << "     %$     goal cell = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")\n" << std::endl;

    GridCell currCell;
    int currValue;
    int minRow, maxRow, minCol, maxCol;
    std::queue<GridCell> waveQ;

    // set goal cell value to 2
    set(goal, 2);
    waveQ.push(goal);

    // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
    // goal is unreachable from start
    while (!waveQ.empty()) {
        currCell = waveQ.front();
        currValue = get(currCell);

        // find free neighbors
        minRow = std::max(currCell.getRow() - 1, 0);
        maxRow = std::min(currCell.getRow() + 1, getGridHeight() - 1);
        minCol = std::max(currCell.getCol() - 1, 0);
        maxCol = std::min(currCell.getCol() + 1, getGridWidth() - 1);
        for (int nRow = minRow; nRow <= maxRow; ++nRow) {
            for (int nCol = minCol; nCol <= maxCol; ++nCol) {
                // if neighbor's value == 0, update the value
                // and enqueue neighbor for next generation of wave
                if (get(nRow, nCol) == 0) {
                    GridCell tempNeighborCell(nRow, nCol);
                    set(tempNeighborCell, currValue + 1);
                    waveQ.push(tempNeighborCell);

                    // once we've enqueued a cell close to the start cell, we're done
                    if (isNear(start, tempNeighborCell)) {
                        return tempNeighborCell;
                    }
                }
            }
        }
        waveQ.pop();
    }
    return goal;
}

/**
 * Optimally Focused Wave Front propagation.
 * Cost(i,j) = Weight(i,j) + heuristic(i,j)
 * Uses Cost(i,j) to determine priority for expansion.
 *
 * @param goal
 * @param start
 * @return true if a path between goal and start is found, false otherwise.
 */
GridCell OccGrid::propOFWF(GridCell goal, GridCell start) {
    std::cout << "\n --+*%$ Waves - Optimally Focused Wave Front" << std::endl;
    std::cout << "     %$    start cell = (c: " << start.getCol() << ", r: " << start.getRow() << ")" << std::endl;
    std::cout << "     %$     goal cell = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")\n" << std::endl;

//    inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);

    GridCell currCell, tempNeighborCell;
    int dx, dy;
    int tempCost, tempHeuristic;
    int currRow, currCol;
    int minRow, maxRow, minCol, maxCol;
    std::priority_queue<std::pair<int, GridCell>,
                        std::vector<std::pair<int, GridCell>>,
                        CompareGreater
                       >
                        waveQ;
    std::pair<double, GridCell> currPair;

    // set goal cell value to 2 and push to waveQ
    set(goal, 2);
    waveQ.push(std::make_pair(2, goal));

    // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
    // goal is unreachable from start
    while (!waveQ.empty()) {
        currPair = waveQ.top();
        currCell = currPair.second;
        currRow = currCell.getRow();
        currCol = currCell.getCol();
//        currWeight = get(currCell);

        // find free neighbors, update weights, calculate costs, push onto priority queue
        minRow = std::max(currRow - 1, 0);
        maxRow = std::min(currRow + 1, getGridHeight() - 1);
        minCol = std::max(currCol - 1, 0);
        maxCol = std::min(currCol + 1, getGridWidth() - 1);
        for (int nRow = minRow; nRow <= maxRow; ++nRow) {
            for (int nCol = minCol; nCol <= maxCol; ++nCol) {
                tempNeighborCell = GridCell(nRow, nCol);
                if (tempNeighborCell.equals(currCell)) {
                    continue;
                }
                // if neighbor's weight == 0, it is free
                if (get(tempNeighborCell) == 0) {
                    setWeightMWF(tempNeighborCell);
                    dx = abs(nCol - start.getCol());
                    dy = abs(nRow - start.getRow());
                    tempHeuristic = 3 * (dx + dy) + (4 - 2 * 3) * std::min(dx, dy);
                    tempCost = get(tempNeighborCell) + tempHeuristic;
                    waveQ.push(std::make_pair(tempCost, tempNeighborCell));

//                    std::cout << "     (" << tempNeighborCell.getCol() << ", " << tempNeighborCell.getRow()
//                              << ") cost = " << tempCost << std::endl;

                    // once we've enqueued a cell near the start cell, we're done
                    if (isNear(start, tempNeighborCell)) {
                        return tempNeighborCell;
                    }

                }
            }
        }
        waveQ.pop();
    }
    return goal;
}

/**
 * Calculates cell's weight according to Modified Wave Front (MWF). Finds the neighboring cell with the lowest weight,
 * then updates its weight accordingly.
 *
 * @param cell
 */
void OccGrid::setWeightMWF(GridCell& cell) {
    std::priority_queue<std::pair<int, GridCell>,
                        std::vector<std::pair<int, GridCell>>,
                        CompareGreater
                       > candidates;
    int minRow, maxRow, minCol, maxCol;
    minRow = std::max(cell.getRow() - 1, 0);
    maxRow = std::min(cell.getRow() + 1, getGridHeight() - 1);
    minCol = std::max(cell.getCol() - 1, 0);
    maxCol = std::min(cell.getCol() + 1, getGridWidth() - 1);
    for (int nRow = minRow; nRow <= maxRow; ++nRow) {
        for (int nCol = minCol; nCol <= maxCol; ++nCol) {
            if (nRow == cell.getRow() && nCol == cell.getCol()) {
                continue;
            }
            if (get(nRow, nCol) >= 2) {
                candidates.push(std::make_pair(get(nRow, nCol), GridCell(nRow, nCol)));
            }
        }
    }
    GridCell minNeighbor = candidates.top().second;
    int minWeight = candidates.top().first;
    if ((cell.getRow() == minNeighbor.getRow()) || (cell.getCol() == minNeighbor.getCol())) {
        set(cell, minWeight + 3);
    } else {
        set(cell, minWeight + 4);
    }
}

/**
 * Tests if GridCells c1 and c2 are near each other. c1 is near c2 if it is within c2's depth-2 square neighborhood.
 * -  -  -  -  -  -  -
 * -  +  +  +  +  +  -
 * -  +  +  +  +  +  -
 * -  +  +  c2 +  +  -
 * -  +  +  +  +  +  -
 * -  +  +  +  +  +  -
 * -  -  -  -  -  -  -
 * 
 * @param c1 
 * @param c2 
 * @return true if c1 and c2 are within each other's depth-2 square neighborhood. otherwise, false
 */
bool OccGrid::isNear(const GridCell& c1, const GridCell& c2) {
    return ((std::abs(c1.getCol() - c2.getCol()) <= 2) && (std::abs(c1.getRow() - c2.getRow()) <= 2));
}

/**
 * Finds the closest open GridCell. Use when a start or goal point lies within obstacles.
 * Basically, searches in the 4 cardinal directions for an open cell.
 *
 * @param cell
 * @return GridCell - nearest free cell
 */
GridCell OccGrid::findClosestFreeCell(GridCell& cell) {
    if (get(cell) == 0) {
        return cell;
    }

    int n{cell.getRow() - 1};
    int w{cell.getCol() - 1};
    int s{cell.getRow() + 1};
    int e{cell.getCol() + 1};

    GridCell north, south, west, east;
    for(int i = 0; i < 150; ++i) {
        north = GridCell(n--, cell.getCol());
        fitInGrid(north);
        south = GridCell(s++, cell.getCol());
        fitInGrid(south);
        west = GridCell(cell.getRow(), w--);
        fitInGrid(west);
        east = GridCell(cell.getRow(), e++);
        fitInGrid(east);

        if (get(north) == 0) {
            return north;
        }
        if (get(south) == 0) {
            return south;
        }
        if (get(west) == 0) {
            return west;
        }
        if (get(east) == 0) {
            return east;
        }
    }
    return cell;
}

/**
 * Normalizes a GridCell. If the GridCell points outside the grid, it gets reassigned to the closest edge cell on the grid.
 * Then, if the GridCell is occupied, it gets reassigned to the closest open cell on the grid. This is so you can find
 * a path between any 2 points in the map.
 *
 * @param cell
 */
void OccGrid::normCell(GridCell& cell) {
    fitInGrid(cell);
    if (get(cell) == 1) {
        cell = findClosestFreeCell(cell);
    }
}

void OccGrid::fitInGrid(GridCell& cell) {
    int row{cell.getRow()};
    int col{cell.getCol()};
    if (row < 0 || row >= GRID_HEIGHT || col < 0 || col >= GRID_WIDTH) {
        std::cout << "Grid index out of bounds. Using closest border cell" << std::endl;
        if (row < 0) {
            cell.setRow(0);
        }
        if (row >= GRID_HEIGHT) {
            cell.setRow(GRID_HEIGHT - 1);
        }
        if (col < 0) {
            cell.setCol(0);
        }
        if (col >= GRID_WIDTH) {
            cell.setCol(GRID_WIDTH - 1);
        }
    }
}




//int main(int argc, char *argv[]) {
//   OccGrid myGrid;
//   myGrid.inputGrid("../bitmaps/2dmap-01.pnm", 5.0);
//   myGrid.outputGrid("../bitmaps/2dmap-scaled.pnm");
//   myGrid.growGrid(0.3);
//   myGrid.outputGrid("../bitmaps/2dmap-grow.pnm");
//   Point goal(124.0, 36.6);
//   GridCell goalCell = myGrid.pointToCell(goal);
//   Point start(4, 63.6);
//   GridCell startCell = myGrid.pointToCell(start);
//   myGrid.propWavesBasic(goalCell, startCell);
//}
