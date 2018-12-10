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
  bool operator()(std::pair<int, GridCell> &p1, std::pair<int, GridCell> &p2) {
    if (p1.first > p2.first) {
      return true;
    } else if (p1.first == p2.first) {
      return p1.second.getCol() > p2.second.getCol();
    } else {
      return false;
    }
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
//  std::cout << "   --+* Map output to " << filename << std::endl;
  outFile.close();
}

void OccGrid::resetGrid() {
  for (auto &row : grid) {
    row.fill(0);
  }
}

int OccGrid::get(int col, int row) {
  return grid[row][col];
}

int OccGrid::get(GridCell cell) {
  int row{cell.getRow()};
  int col{cell.getCol()};
  return OccGrid::get(col, row);
}

void OccGrid::set(GridCell cell, int value) {
  int row{cell.getRow()};
  int col{cell.getCol()};
  grid[row][col] = value;
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
  std::array<std::array<int, GRID_WIDTH>, GRID_HEIGHT> result{{{0}}};

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
GridCell OccGrid::propWavesBasic(GridCell &goal, GridCell &start) {
  std::cout << "\n+--+*%$ Waves - 8-cell, square neighborhood" << std::endl;
  std::cout << "       start = (c: " << start.getCol() << ", r: " << start.getRow() << ")";
  std::cout << "    goal = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")\n" << std::endl;

  GridCell currCell;
  std::queue<GridCell> waveQ;

  // set goal cell value to 2
  set(goal, 2);
  waveQ.push(goal);

  // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
  // goal is unreachable from start
  while (!waveQ.empty()) {
    currCell = waveQ.front();

    // find free neighbors, update weights, calculate costs, push onto priority queue
    for (auto &neighbor : getNeighbors(currCell, 1)) {
      if (get(neighbor) == 0) {
        setWeight(neighbor, 1, 1);
        waveQ.emplace(neighbor);

        // once we've enqueued a cell near the start cell, we're done
//        if (isNear(start, neighbor)) {
        if (start == neighbor) {
          return neighbor;
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
GridCell OccGrid::propOFWF(GridCell &goal, GridCell &start) {
  std::cout << "\n+--+*%$ Waves - Optimally Focused Wave Front" << std::endl;
  std::cout << "        start = (c: " << start.getCol() << ", r: " << start.getRow() << ")";
  std::cout << "    goal = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")" << std::endl;

  std::priority_queue<std::pair<int, GridCell>,
                      std::vector<std::pair<int, GridCell>>,
                      CompareGreater>
      waveQ;

  // set goal cell value to 2 and push to waveQ
  set(goal, 2);
  waveQ.emplace(2, goal);

  GridCell currCell;
  int dx, dy;
  int cost, heuristic;

  // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
  // goal is unreachable from start
  while (!waveQ.empty()) {
    currCell = waveQ.top().second;

    // find free neighbors, update weights, calculate costs, push onto priority queue
    for (auto &neighbor : getNeighbors(currCell, 0)) {
      if (get(neighbor) == 0) {
        setWeight(neighbor, 3, 4);
        dx = abs(neighbor.getCol() - start.getCol());
        dy = abs(neighbor.getRow() - start.getRow());
        heuristic = 3 * (dx + dy) + (4 - 2 * 3) * std::min(dx, dy);
        cost = get(neighbor) + heuristic;
        waveQ.emplace(cost, neighbor);

        // once we've enqueued a cell near the start cell, we're done
        if (start == neighbor) {
          return neighbor;
        }
      }
    }
    waveQ.pop();
  }
  return goal;
}


/**
 * Calculates cell's weight. Finds the neighboring cell with the lowest weight, then updates its weight
 * based on if it is orthogonal or diagonal to the min neighbor.
 *
 * @param cell
 */
void OccGrid::setWeight(GridCell &cell, int orthoDist, int diagDist) {
  // find neighbor with minimum weight
  GridCell minNeighbor {};
  int minSoFar {INT_MAX};
  int currWeight;
  for (const auto &neighbor : getNeighbors(cell, 0)) {
    currWeight = get(neighbor);
    if ((currWeight < minSoFar) && (currWeight > 1)) {
      minSoFar = currWeight;
      minNeighbor = neighbor;
    }
  }
  if ((cell.getRow() == minNeighbor.getRow()) || (cell.getCol() == minNeighbor.getCol())) {
    set(cell, minSoFar + orthoDist);
  } else {
    set(cell, minSoFar + diagDist);
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
bool OccGrid::isNear(const GridCell &c1, const GridCell &c2) {
  return ((std::abs(c1.getCol() - c2.getCol()) <= 2) && (std::abs(c1.getRow() - c2.getRow()) <= 2));
}

/**
 * Finds the closest open GridCell. Use when a start or goal point lies within obstacles.
 * Basically, searches in the 4 cardinal directions for an open cell.
 *
 * @param cell
 * @return GridCell - nearest free cell
 */
GridCell OccGrid::findClosestFreeCell(GridCell &cell) {
  if (get(cell) == 0) {
    return cell;
  }

  std::pair<int, GridCell> northDist = findFree(cell, "north");
  std::pair<int, GridCell> westDist = findFree(cell, "west");
  std::pair<int, GridCell> southDist = findFree(cell, "south");
  std::pair<int, GridCell> eastDist = findFree(cell, "east");
  std::vector<std::pair<int, GridCell>> distCells {};
  distCells.emplace_back(northDist);
  distCells.emplace_back(southDist);
  distCells.emplace_back(westDist);
  distCells.emplace_back(eastDist);

  int minSoFar = INT_MAX;
  GridCell minCellSoFar {cell};
  for (auto &distCell : distCells) {
    if (distCell.first < minSoFar) {
      minCellSoFar = distCell.second;
    }
  }
  return minCellSoFar;
}


std::pair<int, GridCell> OccGrid::findFree(GridCell &cell, std::string direction) {
  int stepCounter = 0;
  int currCol {cell.getCol()};
  int currRow {cell.getRow()};

  while (get(currCol, currRow) != 0) {
    if (direction == "north") {
      currRow--;
    } else if (direction == "south") {
      currRow++;
    } else if (direction == "west") {
      currCol--;
    } else if (direction == "east") {
      currCol++;
    }
    stepCounter++;

    if ((currRow < 0) || (currRow == GRID_HEIGHT) || (currCol < 0) || (currCol == GRID_WIDTH)) {
      stepCounter = INT_MAX;
      break;
    }
  }
  std::pair<int, GridCell> freeCell =  std::make_pair(stepCounter, GridCell(currCol, currRow));
  return freeCell;
}


/**
 * Normalizes a GridCell. If the GridCell points outside the grid, it gets reassigned to the closest edge cell on the grid.
 * Then, if the GridCell is occupied, it gets reassigned to the closest open cell on the grid. This is so you can find
 * a path between any 2 points in the map.
 *
 * @param cell
 */
void OccGrid::normCell(GridCell &cell) {
  fitInGrid(cell);
  if (get(cell) == 1) {
    cell = findClosestFreeCell(cell);
  }
}


void OccGrid::fitInGrid(GridCell &cell) {
  int row {cell.getRow()};
  int col {cell.getCol()};
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


/**
 * Returns all neighboring cells. For layers == 1, it is the 8 immediate neighbors
 *
 * @param cell
 * @param layers - how many layers of neighbors to return
 * @return
 */
std::vector<GridCell> OccGrid::getNeighbors(const GridCell &cell, int layers) {
  std::vector<GridCell> neighbors(static_cast<unsigned long long int>(8 * layers));
  int col {cell.getCol()};
  int row {cell.getRow()};

  int minRow {std::max(row - layers, 0)};
  int maxRow {std::min(row + layers, getGridHeight() - 1)};
  int minCol {std::max(col - layers, 0)};
  int maxCol {std::min(col + layers, getGridWidth() - 1)};
  for (int tempRow = minRow; tempRow <= maxRow; ++tempRow) {
    for (int tempCol = minCol; tempCol <= maxCol; ++tempCol) {
      if (!((tempCol == col) && (tempRow == row))) {
        neighbors.emplace_back(GridCell(tempCol, tempRow));
      }
    }
  }
  return neighbors;
}


// MWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW
//                  Conversion Functions
// MWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW

GridCell pointToCell(const Point &pt) {
  return GridCell(xToCol(pt.getX()), yToRow(pt.getY()));
}


GPS pointToGPS(const Point &pt) {
  return GPS(xToLongitude(pt.getX()), yToLatitude(pt.getY()));
}


Point cellToPoint(const GridCell &cell) {
  return Point(colToX(cell.getCol()), rowToY(cell.getRow()));
}


GPS cellToGPS(const GridCell &cell) {
  return pointToGPS(Point(colToX(cell.getCol()), rowToY(cell.getRow())));
}


GridCell gpsToCell(const GPS &gps) {
  GridCell c = pointToCell(gpsToPoint(gps));
  std::cout << gps.toString() << " --> " << c.toString() << std::endl;
  return pointToCell(gpsToPoint(gps));
}


Point gpsToPoint(const GPS &gps) {
  double longitude = gps.getLong();
  double latitude = gps.getLat();
  double widthLong = MAX_LONG - MIN_LONG;
  double heightLat = MAX_LAT - MIN_LAT;
  double x = ((longitude - MIN_LONG) / widthLong) * WIDTH_METERS;
  double y = ((MAX_LAT - latitude) / heightLat) * HEIGHT_METERS;
  return Point(x, y);
}


int xToCol(double x) {
  return static_cast<int>((static_cast<double>(GRID_WIDTH) / WIDTH_METERS) * x);
}


int yToRow(double y) {
  return static_cast<int>((static_cast<double>(GRID_HEIGHT) / HEIGHT_METERS) * y);
}


double colToX(int n) {
  double metersPerCol = WIDTH_METERS / static_cast<double>(GRID_WIDTH);
  return metersPerCol * n;
}


double rowToY(int m) {
  double metersPerRow = HEIGHT_METERS / static_cast<double>(GRID_HEIGHT);
  return metersPerRow * m;
}


double xToLongitude(double x) {
  double widthLong = MAX_LONG - MIN_LONG;
  double longitude = ((x / WIDTH_METERS) * widthLong) + MIN_LONG;
  return longitude;
}


double yToLatitude(double y) {
  double heightLat = MAX_LAT - MIN_LAT;
  double latitude = ((1 - (y / HEIGHT_METERS)) * heightLat) + MIN_LAT;
  return latitude;
}


double longitudeToX(double longitude) {
  return WIDTH_METERS * (longitude - MIN_LONG) / (MAX_LONG - MIN_LONG);
}


double latitudeToY(double latitude) {
  return HEIGHT_METERS * (MAX_LAT - latitude) / (MAX_LAT - MIN_LAT);
}
