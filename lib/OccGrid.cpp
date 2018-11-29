/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: OccGrid.cpp
 */

#include <array>
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

void OccGrid::inputGrid(std::string filename, double mapScale = SCALE_MAP) {
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
    if (row > GRID_HEIGHT || col > GRID_WIDTH) {
        std::cerr << "Grid index out of bounds." << std::endl;
        exit(1);
    }
    return grid[row][col];
}

int OccGrid::get(GridCell cell) {
    return grid[cell.getRow()][cell.getCol()];
}

void OccGrid::set(GridCell cell, int value) {
    grid[cell.getRow()][cell.getCol()] = value;
}

GridCell OccGrid::pointToCell(Point pt) {
    return GridCell(yToRow(pt.getY()), xToCol(pt.getX()));
}

Point OccGrid::cellToPoint(GridCell cell) {
    return Point(colToX(cell.getCol()), rowToY(cell.getRow()));
}

GridCell OccGrid::coordToCell(Point coord) {
    return pointToCell(coordToPoint(coord));
}

Point OccGrid::coordToPoint(Point coord) {
    double longitude, latitude, x, y;
    longitude = coord.getX();
    latitude = coord.getY();
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
 * @return true if a path between goal and start is found. false otherwise.
 */
bool OccGrid::propWavesBasic(GridCell goal, GridCell start) {
    std::cout << "\n --+*%$ Waves - 8-cell, square neighborhood" << std::endl;
    std::cout << "     %$    start cell = (c: " << start.getCol() << ", r: " << start.getRow() << ")" << std::endl;
    std::cout << "     %$     goal cell = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")\n" << std::endl;

//    inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);

    bool reachable(false);
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

                    // once we've enqueued the start cell, we're done
                    if (start.equals(tempNeighborCell)) {
                        reachable = true;
                        return reachable;
                    }
                }
            }
        }
        waveQ.pop();
    }
    return reachable;
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
bool OccGrid::propOFWF(GridCell goal, GridCell start) {
    std::cout << "\n --+*%$ Waves - Optimally Focused Wave Front" << std::endl;
    std::cout << "     %$    start cell = (c: " << start.getCol() << ", r: " << start.getRow() << ")" << std::endl;
    std::cout << "     %$     goal cell = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")\n" << std::endl;

//    inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);

    bool reachable(false);
    GridCell currCell, tempNeighborCell;
    double dx, dy;
    int currWeight;
    double tempCost, tempHeuristic;
    int currRow, currCol;
    int minRow, maxRow, minCol, maxCol;
    std::priority_queue<std::pair<double, GridCell>,
                        std::vector<std::pair<double, GridCell>>,
                        CompareGreater
                       >
                       waveQ;
    std::pair<double, GridCell> currPair;

    // set goal cell value to 2
    currCell = goal;
    set(goal, 2);
    dx = static_cast<double>(std::abs(currCell.getCol() - start.getCol()));
    dy = static_cast<double>(std::abs(currCell.getRow() - start.getRow()));
    tempHeuristic = 3 * ((dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy));
    tempCost = 2 + tempHeuristic;
    waveQ.push(std::make_pair(tempCost, goal));

    // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
    // goal is unreachable from start
    while (!waveQ.empty()) {
        currPair = waveQ.top();
        currCell = currPair.second;
        currRow = currCell.getRow();
        currCol = currCell.getCol();
        currWeight = get(currCell);

//        std::cout << "expanding node (" << currCell.getCol() << ", " << currCell.getRow() << ") -- weight = " << currWeight << std::endl;

        // find free neighbors, update weights, calculate costs, push onto priority queue
        minRow = std::max(currRow - 1, 0);
        maxRow = std::min(currRow + 1, getGridHeight() - 1);
        minCol = std::max(currCol - 1, 0);
        maxCol = std::min(currCol + 1, getGridWidth() - 1);
        for (int nRow = minRow; nRow <= maxRow; ++nRow) {
            for (int nCol = minCol; nCol <= maxCol; ++nCol) {
                // if neighbor's weight == 0, it is free
                if (get(nRow, nCol) == 0) {
                    tempNeighborCell = GridCell(nRow, nCol);
                    setWeightMWF(tempNeighborCell);

                    std::cout << "(" << nCol << ", " << nRow << ") set to " << get(tempNeighborCell);

                    dx = std::abs(nCol - start.getCol());
                    dy = std::abs(nRow - start.getRow());
                    tempHeuristic = 3 * (static_cast<double>(dx + dy) + (std::sqrt(2) - 2) * static_cast<double>(std::min(dx, dy)));
                    tempCost = static_cast<double>(get(tempNeighborCell)) + tempHeuristic;
                    waveQ.push(std::make_pair(tempCost, tempNeighborCell));

                    std::cout << "     (" << tempNeighborCell.getCol() << ", " << tempNeighborCell.getRow() << ") cost = " << tempCost << std::endl;

                    // once we've enqueued the start cell, we're done
                    if (start.equals(tempNeighborCell)) {
                        reachable = true;
                        return reachable;
                    }

                }
            }
        }
        waveQ.pop();
    }
    return reachable;
}

/**
 * Calculates cell's weight according to Modified Wave Front (MWF)
 * @param cell
 */
void OccGrid::setWeightMWF(GridCell cell) {
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
