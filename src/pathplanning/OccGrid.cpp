/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: OccGrid.cpp
 */

#include "../../include/pathplanning/OccGrid.h"


class CompareGreater {
public:
  bool operator()(std::pair<double, GridCell> &p1, std::pair<double, GridCell> &p2) {
    if (p1.first > p2.first) {
      return true;
    } else if (p1.first - p2.first == 0) {
      return p1.second.getCol() > p2.second.getCol();
    } else {
      return false;
    }
  }
};


OccGrid::OccGrid() {
  grid.resize(GRID_HEIGHT);
  for (auto &row : grid) {
    row.resize(GRID_WIDTH, 0);
  }
}


OccGrid::OccGrid(const std::string &filename, double mapScale = SCALE_MAP) {
  grid.resize(GRID_HEIGHT);
  for (auto &row : grid) {
    row.resize(GRID_WIDTH, 0);
  }

  long inRow, inCol;
  long inputWidth, inputHeight, maxVal;
  int currValue;
  std::string fileFormat;
  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ifstream::in);

  /* Read past first line */
  ifs >> fileFormat;
  /* Read in inputWidth, inputHeight, maxVal */
  ifs >> inputWidth >> inputHeight >> maxVal;

  if (fileFormat == "P5") {
    char nextChar;
    for (inRow = 0; inRow < inputHeight; ++inRow) {
      for (inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextChar;
        currValue = grid[static_cast<long>(inRow / mapScale)][static_cast<long>(inCol / mapScale)];
        if (currValue == 1) {
          continue;
        }
        // char(-1) == 255, which denotes an unoccupied pixel in input image
        if (nextChar != char(-1)) {  // cell is occupied
          grid[static_cast<long>(inRow / mapScale)][static_cast<long>(inCol / mapScale)] = 1;
        }
      }
    }
  }

  if (fileFormat == "P2") {
    long nextLong;
    for (inRow = 0; inRow < inputHeight; ++inRow) {
      for (inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextLong;
        if (nextLong == 0) {
          grid[static_cast<long>(inRow / mapScale)][static_cast<long>(inCol / mapScale)] = 1;
        }
      }
    }
  }
  ifs.close();
}


OccGrid::~OccGrid() = default;


long OccGrid::get(long col, long row) const {
  return grid[row][col];
}


long OccGrid::get(const GridCell &cell) const {
  return OccGrid::get(cell.getCol(), cell.getRow());
}


void OccGrid::set(const GridCell &cell, long value) {
  grid[cell.getRow()][cell.getCol()] = value;
}


void OccGrid::convertToDebugGrid() {
  for (long row = 0; row < GRID_HEIGHT; ++row) {
    for (long col = 0; col < GRID_WIDTH; ++col) {
      if (grid[row][col] == 0) {
        grid[row][col] = 255;
      }
      if (grid[row][col] == 1) {
        grid[row][col] = 0;
      }
    }
  }
}

/**
 * Outputs the OccGrid as a .pnm bitmap where occupied cells are black, open cells are white.
 * 
 * @param filename 
 */
void OccGrid::outputGrid(const std::string& filename) {
  std::ofstream outFile(filename.c_str(), std::ofstream::out);
  outFile << "P2" << std::endl;
  outFile << GRID_WIDTH << " " << GRID_HEIGHT << std::endl << 255 << std::endl;

  for (long row = 0; row < GRID_HEIGHT; ++row) {
    for (long col = 0; col < GRID_WIDTH; ++col) {
      if (grid[row][col] == 1) {
        outFile << 0 << " ";
      } else if (grid[row][col] == 0) {
        outFile << 255 << " ";
      } else {
        outFile << grid[row][col] << " ";
      }
    }
  }
//  std::cout << "   --+* Map output to " << filename << std::endl;
  outFile.close();
}


void OccGrid::outputDebugGrid(const std::string& filename) {
  std::ofstream outFile(filename.c_str(), std::ofstream::out);
  outFile << "P2" << std::endl;
  outFile << GRID_WIDTH << " " << GRID_HEIGHT << std::endl << 255 << std::endl;

  for (long row = 0; row < GRID_HEIGHT; ++row) {
    for (long col = 0; col < GRID_WIDTH; ++col) {
      outFile << grid[row][col] << " ";
    }
  }
//  std::cout << "   --+* Map output to " << filename << std::endl;
  outFile.close();
}


void OccGrid::outputWaves(const std::string &filename, const std::string &color) {
  std::ofstream outFile(filename.c_str(), std::ofstream::out);
  outFile << "P3" << std::endl;
  outFile << GRID_WIDTH << " " << GRID_HEIGHT << std::endl << 255 << std::endl;

  long maxDistance = findMaxDistance();
//  double tintMultiplier = 255.0 / maxDistance;
  long tintColor;

  for (long row = 0; row < GRID_HEIGHT; ++row) {
    for (long col = 0; col < GRID_WIDTH; ++col) {
      if (grid[row][col] == 1) {
        outFile << 0 << " " << 0 << " " << 0 << " ";
      } else if (grid[row][col] == 0) {
        outFile << 255 << " " << 255 << " " << 255 << " ";
      } else {
        tintColor = static_cast<long>(((std::trunc(10.0 * (maxDistance - grid[row][col]) / maxDistance)) / 10.0) * 255);
        if (color == "R") {
          outFile << 255 << " " << tintColor << " " << tintColor << " ";
        }
        if (color == "G") {
          outFile << tintColor << " " << 255 << " " << tintColor << " ";
        }
        if (color == "B") {
          outFile << tintColor << " " << tintColor << " " << 255 << " ";
        }
      }
    }
  }
  outFile.close();
}

/**
 * Expands obstacle boundaries by a certain radius. This is done so that the robot
 * can be represented as a Point rather than a 3 dimensional object.
 * @param radius - distance in meters to expand obstacles.
 */
OccGrid OccGrid::growGrid(double radius) {
  // calculate # of grid cells to use for radius
  long growH, growV, growSize;
  double ratioH, ratioV;
  ratioV = static_cast<double>(GRID_HEIGHT) / HEIGHT_METERS;
  ratioH = static_cast<double>(GRID_WIDTH) / WIDTH_METERS;
  growV = static_cast<long>(ceil(radius * ratioV));
  growH = static_cast<long>(ceil(radius * ratioH));
  growSize = std::max(growH, growV);

  // create a new 2d array to hold the result of grow.
  OccGrid result;
  std::vector<GridCell> neighborhood;
  neighborhood.reserve((4 * growSize) * (growSize + 1) + 1);

  // loop through grid, looking for 1, then growing them on result
  long row, col;
  for (row = 0; row != GRID_HEIGHT; ++row) {
    for (col = 0; col != GRID_WIDTH; ++col) {
      if (grid[row][col] == 1) {
        getNeighborhood(GridCell(col, row), growSize, neighborhood);
        for (const auto &neighbor : neighborhood) {
          result.set(neighbor, 1);
        }
        neighborhood.clear();
      }
    }
  }
  // deep copy values from result long grid
  return result;
}


/**
 * Basic wave propagation. 8-cell, square neighborhoods with wave values differing by 1.
 *
 * @param goal
 * @param start
 * @return If path is found, return the GridCell that the search stopped on. If path is not found, return goal.
 */
std::pair<GridCell, long> OccGrid::propWavesBasic(GridCell &goal, GridCell &start, long orthoDist, long diagDist) {
//  std::cout << "\n+--+*%$ Waves - 8-cell, square neighborhood" << std::endl;
//  std::cout << "       start = (c: " << start.getCol() << ", r: " << start.getRow() << ")";
//  std::cout << "    goal = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")\n" << std::endl;

  std::queue<GridCell> waveQ;

  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);

  // set goal cell value to 2
  set(goal, 2);
  waveQ.push(goal);

  GridCell currCell;

  // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
  // goal is unreachable from start
  long numCellsVisited = 0;
  while (!waveQ.empty()) {
    currCell = waveQ.front();

    // find free neighbors, update weights, calculate costs, push onto queue
    getNeighborhood(currCell, 1, neighborhood);
    for (auto &neighbor : neighborhood) {
      if (get(neighbor) == 0) {
        setWeight(neighbor, orthoDist, diagDist);
        waveQ.emplace(neighbor);
        numCellsVisited++;

        // once we've enqueued the start cell, we're done
        if (start.equals(neighbor)) {
          return std::make_pair(neighbor, numCellsVisited);
        }
      }
    }
    neighborhood.clear();
    waveQ.pop();
  }
  return std::make_pair(currCell, numCellsVisited);
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
std::pair<GridCell, long> OccGrid::propOFWF(GridCell &goal, GridCell &start, long orthoDist, long diagDist) {
//  std::cout << "\n+--+*%$ Waves - Optimally Focused Wave Front" << std::endl;
//  std::cout << "        start = (c: " << start.getCol() << ", r: " << start.getRow() << ")";
//  std::cout << "    goal = (c: " << goal.getCol() << ", r: " << goal.getRow() << ")" << std::endl;

  std::priority_queue<std::pair<double, GridCell>,
                      std::vector<std::pair<double, GridCell>>,
                      CompareGreater>
      waveQ;

  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);

  // set goal cell value to 2 and push to waveQ
  set(goal, 2);
  waveQ.emplace(2, goal);

  GridCell currCell;
  double dx, dy;
  double cost, heuristic;

  // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
  // goal is unreachable from start
  long numCellsVisited {0};
  while (!waveQ.empty()) {
    currCell = waveQ.top().second;
//    numCellsVisited++;

    // find free neighbors, update weights, calculate costs, push onto priority queue
    getNeighborhood(currCell, 1, neighborhood);
    for (auto &neighbor : neighborhood) {
      if (get(neighbor) == 0) {
        setWeight(neighbor, orthoDist, diagDist);
        dx = static_cast<double>(std::labs(neighbor.getCol() - start.getCol()));
        dy = static_cast<double>(std::labs(neighbor.getRow() - start.getRow()));
//        heuristic = static_cast<double>(orthoDist * (dx + dy) + (diagDist - 2.0 * orthoDist) * std::min(dx, dy)) / 1.00001;
//        heuristic = (orthoDist * (dx + dy) + (diagDist - 2.0 * orthoDist) * std::min(dx, dy)) / 1.00001;
//        heuristic = (orthoDist * (dx + dy) + (diagDist - 2.0 * orthoDist) * std::min(dx, dy)) * 0.99;
        heuristic = 0.99 * orthoDist * ((dx + dy) + (sqrt(2.0) - 2) * std::min(dx, dy));
        cost = static_cast<double>(get(neighbor)) + heuristic;

        // Use cost to set priority of neighbor.
        waveQ.emplace(cost, neighbor);
        numCellsVisited++;

        // once we've enqueued the start cell, we're done
        if (start.equals(neighbor)) {
          return std::make_pair(neighbor, numCellsVisited);
        }
      }
    }
    neighborhood.clear();
    waveQ.pop();
  }
  return std::make_pair(currCell, numCellsVisited);
}


/**
 * Calculates cell's weight. Finds the neighboring cell with the lowest weight, then updates its weight
 * based on if it is orthogonal or diagonal to the min neighbor.
 *
 * @param cell
 */
void OccGrid::setWeight(GridCell &cell, long orthoDist, long diagDist) {
  // find neighbor with minimum weight
  GridCell minNeighbor;
  long minSoFar = LONG_MAX;
  long currWeight;

  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);
  getNeighborhood(cell, 1, neighborhood);

  for (const auto &neighbor : neighborhood) {
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
 * Finds the closest open GridCell. Use when a start or goal Point lies within obstacles.
 * Basically, searches in the 4 cardinal directions for an open cell.
 *
 * @param cell
 * @return GridCell - nearest free cell
 */
GridCell OccGrid::findClosestFreeCell(GridCell &cell) {
  if (get(cell) == 0) {
    return cell;
  }

  std::pair<long, GridCell> northDist = findFree(cell, "north");
  std::pair<long, GridCell> westDist = findFree(cell, "west");
  std::pair<long, GridCell> southDist = findFree(cell, "south");
  std::pair<long, GridCell> eastDist = findFree(cell, "east");
  std::vector<std::pair<long, GridCell>> distCells {};
  distCells.emplace_back(northDist);
  distCells.emplace_back(southDist);
  distCells.emplace_back(westDist);
  distCells.emplace_back(eastDist);

  long minSoFar = LONG_MAX;
  GridCell minCellSoFar {cell};
  for (auto &distCell : distCells) {
    if (distCell.first < minSoFar) {
      minSoFar = distCell.first;
      minCellSoFar = distCell.second;
    }
  }
  return minCellSoFar;
}


std::pair<long, GridCell> OccGrid::findFree(GridCell &cell, const std::string& direction) {
  long stepCounter = 0;
  long currCol {cell.getCol()};
  long currRow {cell.getRow()};

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
      stepCounter = LONG_MAX;
      break;
    }
  }
  std::pair<long, GridCell> freeCell = std::make_pair(stepCounter, GridCell(currCol, currRow));
  return freeCell;
}


/**
 * Normalizes a GridCell. If the GridCell Points outside the grid, it gets reassigned to the closest edge cell on the grid.
 * Then, if the GridCell is occupied, it gets reassigned to the closest open cell on the grid. This is so you can find
 * a path between any 2 Points in the map.
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
  long row {cell.getRow()};
  long col {cell.getCol()};
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


long OccGrid::findMaxDistance() const {
  std::vector<long> rowMaxValues;
  rowMaxValues.reserve(GRID_HEIGHT);
  for (auto &row : grid) {
    rowMaxValues.emplace_back(*std::max_element(row.begin(), row.end()));
  }

  return *std::max_element(rowMaxValues.begin(), rowMaxValues.end());
}


/**
 * Returns a vector<GridCell> of a cell's neighborhood (includes cell itself).
 * For layers == 1, a neighborhood includes 9 cells (8 immediate neighbors + cell itself)
 *
 * @param cell
 * @param layers - how many layers of neighbors to return
 * @return
 */
void OccGrid::getNeighborhood(const GridCell &cell,
                              long layers,
                              std::vector<GridCell> &neighborhood) {
  long col = cell.getCol();
  long row = cell.getRow();

  long minRow = std::max(row - layers, 0l);
  long maxRow = std::min(row + layers, GRID_HEIGHT - 1l);
  long minCol = std::max(col - layers, 0l);
  long maxCol = std::min(col + layers, GRID_WIDTH - 1l);
  for (long tempRow = minRow; tempRow <= maxRow; ++tempRow) {
    for (long tempCol = minCol; tempCol <= maxCol; ++tempCol) {
      neighborhood.emplace_back(GridCell(tempCol, tempRow));
    }
  }
}


/**
 * Checks to see if there is a clear line of sight between c0 and c1.
 *
 * @param c0
 * @param c1
 * @return true if there is a clear line of sight between c0 and c1
 */
bool OccGrid::isInLine(const GridCell &c0, const GridCell &c1) {
  // check if any cell in the line between c0 and c1 is occupied by an obstacle.
  for (const auto &gc : drawLine(c0, c1)) {
    if (get(gc) == 1) {
      return false;
    }
  }
  return true;
}


/**
 * Bresenham's algorithm for approximating a straight line between 2 cells on a grid.
 *
 * @param cell0 source GridCell
 * @param cell1 destination GridCell
 * @return std::vector<GridCell> vector of GridCells, approximating a straight line between c0 and c1
 */
std::vector<GridCell> OccGrid::drawLine(const GridCell &cell0, const GridCell &cell1) {
  const long c0 {cell0.getCol()};
  const long c1 {cell1.getCol()};
  const long r0 {cell0.getRow()};
  const long r1 {cell1.getRow()};
  const long dCol {llabs(c1 - c0)};
  const long dRow {llabs(r1 - r0)};

  std::vector<GridCell> lineOfCells;

  if (dRow < dCol) {
    if (c0 > c1) {
      lineOfCells = drawLineLow(c1, r1, c0, r0);
    } else {
      lineOfCells = drawLineLow(c0, r0, c1, r1);
    }
  } else {
    if (r0 > r1) {
      lineOfCells = drawLineHigh(c1, r1, c0, r0);
    } else {
      lineOfCells = drawLineHigh(c0, r0, c1, r1);
    }
  }
  return lineOfCells;
}


/**
 * Helper function for drawLine for cases where the slope of the line is < 1.
 *
 * @param c0
 * @param r0
 * @param c1
 * @param r1
 * @return
 */
std::vector<GridCell> OccGrid::drawLineLow(const long c0, const long r0, const long c1, const long r1) {
  long dCol = c1 - c0;
  long dRow = r1 - r0;
  long rowIncrement = 1;
  if (dRow < 0) {
    rowIncrement = -1;
    dRow *= -1;
  }
  long error = 2 * dRow - dCol;
  long currRow = r0;

  std::vector<GridCell> lineOfCells;
  lineOfCells.reserve(c1 - c0 + 1);

  for (long currCol = c0; currCol <= c1; ++currCol) {
    lineOfCells.emplace_back(GridCell(currCol, currRow));
    if (error > 0) {
      currRow = currRow + rowIncrement;
      error = error - 2 * dCol;
    }
    error = error + 2 * dRow;
  }

  return lineOfCells;
}


/**
 * Helper function for drawLine for cases where the slope of the line is > 1 (steep).
 *
 * @param c0
 * @param r0
 * @param c1
 * @param r1
 * @return
 */
std::vector<GridCell> OccGrid::drawLineHigh(const long c0, const long r0, const long c1, const long r1) {
  long dCol = c1 - c0;
  long dRow = r1 - r0;
  long colIncrement = 1;
  if (dCol < 0) {
    colIncrement = -1;
    dCol *= -1;
  }
  long error = 2 * dCol - dRow;
  long currCol = c0;

  std::vector<GridCell> lineOfCells;
  lineOfCells.reserve(r1 - r0 + 1);

  for (long currRow = r0; currRow <= r1; ++currRow) {
    lineOfCells.emplace_back(GridCell(currCol, currRow));
    if (error > 0) {
      currCol = currCol + colIncrement;
      error = error - 2 * dRow;
    }
    error = error + 2 * dCol;
  }

  return lineOfCells;
}


double OccGrid::euclideanDist(const GridCell &c0, const GridCell &c1) {
  long dRow = c0.getRow() - c1.getRow();
  long dCol = c0.getCol() - c1.getCol();
  auto dRowSq = static_cast<double>(dRow * dRow);
  auto dColSq = static_cast<double>(dCol * dCol);
  return sqrt(dRowSq + dColSq);
}


void OccGrid::markCell(const GridCell &cell, long value) {
  set(cell, value);
}


void OccGrid::markCells(const std::list<GridCell> &cells, long value) {
  for (const auto &cell : cells) {
    set(cell, value);
  }
}


void OccGrid::markCells(const std::vector<GridCell> &cells, long value) {
  for (const auto &cell : cells) {
    set(cell, value);
  }
}


// MWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW
//                  Conversion Functions
// MWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMWMW

GridCell pointToCell(const Point &pt) {
  return GridCell(xToCol(pt.x()), yToRow(pt.y()));
}

GPS pointToGPS(const Point &pt) {
  return GPS(xToLongitude(pt.x()), yToLatitude(pt.y()));
}

Point cellToPoint(const GridCell &cell) {
  return Point(colToX(cell.getCol()), rowToY(cell.getRow()));
}

GPS cellToGPS(const GridCell &cell) {
  return pointToGPS(cellToPoint(cell));
}

GridCell gpsToCell(const GPS &gps) {
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

long xToCol(double x) {
  return static_cast<long>((static_cast<double>(GRID_WIDTH) / WIDTH_METERS) * x);
}

long yToRow(double y) {
  return static_cast<long>((static_cast<double>(GRID_HEIGHT) / HEIGHT_METERS) * y);
}

double colToX(long n) {
  double metersPerCol = WIDTH_METERS / static_cast<double>(GRID_WIDTH);
  return metersPerCol * (n + 0.5);
}

double rowToY(long m) {
  double metersPerRow = HEIGHT_METERS / static_cast<double>(GRID_HEIGHT);
  return metersPerRow * (m + 0.5);
}

double xToLongitude(double x) {
  double widthLong = MAX_LONG - MIN_LONG;
  return ((x / WIDTH_METERS) * widthLong) + MIN_LONG;
}

double yToLatitude(double y) {
  double heightLat = MAX_LAT - MIN_LAT;
  return ((1 - (y / HEIGHT_METERS)) * heightLat) + MIN_LAT;
}

double longitudeToX(double longitude) {
  return WIDTH_METERS * (longitude - MIN_LONG) / (MAX_LONG - MIN_LONG);
}

double latitudeToY(double latitude) {
  return HEIGHT_METERS * (MAX_LAT - latitude) / (MAX_LAT - MIN_LAT);
}



