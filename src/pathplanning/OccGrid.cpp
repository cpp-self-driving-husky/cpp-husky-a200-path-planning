/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: OccGrid.cpp
 */

#include "pathplanning/OccGrid.h"

namespace pathplanner {

class CompareGreater {
public:
  bool operator()(std::pair<int, GridCell> &p1, std::pair<int, GridCell> &p2) {
    if (p1.first == p2.first) {
      std::hash<int> firstHash;
      return firstHash(p1.first) > firstHash(p2.first);
    }
    return p1.first > p2.first;
  }
};


OccGrid::OccGrid() {
  gridWidth_ = GRID_WIDTH;
  gridHeight_ = GRID_HEIGHT;
  grid_.resize(gridHeight_);
  for (auto &row : grid_) {
    row.resize(gridWidth_, 0);
  }
}


OccGrid::OccGrid(const std::string &filename, double mapScale = SCALE_MAP) {
  std::string fileFormat;
  int inputWidth, inputHeight, maxVal;
  int gridRow, gridCol;

  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ifstream::in);
  ifs >> fileFormat;
  ifs >> inputWidth >> inputHeight;
  gridWidth_ = static_cast<int>(std::ceil(inputWidth / mapScale));
  gridHeight_ = static_cast<int>(std::ceil(inputHeight / mapScale));
  grid_.resize(gridHeight_);
  for (auto &row : grid_) {
    row.resize(gridWidth_, 0);
  }

  if (fileFormat == "P1") {
    char nextChar;
    int nextInt;
    for (int inRow = 0; inRow < inputHeight; ++inRow) {
      for (int inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextChar;
        nextInt = nextChar - '0';
        gridRow = static_cast<int>(inRow / mapScale);
        gridCol = static_cast<int>(inCol / mapScale);
        if (grid_[gridRow][gridCol] == 0) {
          grid_[gridRow][gridCol] = nextInt;
        }
      }
    }
  } else if (fileFormat == "P2") {
    // Read in inputWidth, inputHeight, maxVal
    ifs >> maxVal;

    int nextInt;
    for (int inRow = 0; inRow < inputHeight; ++inRow) {
      for (int inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextInt;
        gridRow = static_cast<int>(inRow / mapScale);
        gridCol = static_cast<int>(inCol / mapScale);
        if (nextInt == 0) {
          grid_[gridRow][gridCol] = 1;
        }
      }
    }
  } else if (fileFormat == "P5") {
    // Read in inputWidth, inputHeight, maxVal
    ifs >> maxVal;

    char nextChar;
    for (int inRow = 0; inRow < inputHeight; ++inRow) {
      for (int inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextChar;
        gridRow = static_cast<int>(inRow / mapScale);
        gridCol = static_cast<int>(inCol / mapScale);
        if ((grid_[gridRow][gridCol] == 0) && (nextChar != char(255))) {
          grid_[gridRow][gridCol] = 1;
        }
      }
    }
  } else {
    std::cout << "Can't read image file format." << std::endl;
  }
  ifs.close();
}


OccGrid::~OccGrid() = default;


int OccGrid::get(const int col, const int row) const {
  return grid_[row][col];
}


int OccGrid::get(const GridCell &cell) const {
  return get(cell.getCol(), cell.getRow());
}


void OccGrid::set(const GridCell &cell, int value) {
  grid_[cell.getRow()][cell.getCol()] = value;
}


/**
 * Outputs the OccGrid as a .pnm bitmap where occupied cells are black, open cells are white.
 * 
 * @param filename 
 */
void OccGrid::outputGrid(const std::string &filename) {
  std::ofstream outFile(filename.c_str(), std::ofstream::out);
  outFile << "P2" << std::endl;
  outFile << gridWidth_ << " " << gridHeight_ << std::endl << 255 << std::endl;

  int currVal;
  for (int row = 0; row < gridHeight_; ++row) {
    for (int col = 0; col < gridWidth_; ++col) {
      currVal = grid_[row][col];
      if (currVal == 1) {
        outFile << 0 << " ";
      }
      if (currVal == 0) {
        outFile << 255 << " ";
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
  // calculate # of grid_ cells to use for radius
  int growH, growV, growSize;
  double ratioH, ratioV;
  ratioV = static_cast<double>(gridHeight_) / HEIGHT_METERS;
  ratioH = static_cast<double>(gridWidth_) / WIDTH_METERS;
  growV = static_cast<int>(ceil(radius * ratioV));
  growH = static_cast<int>(ceil(radius * ratioH));
  growSize = std::max(growH, growV);

  // create a new 2d array to hold the result of grow.
  OccGrid result;
  std::vector<GridCell> neighborhood;
  neighborhood.reserve((4 * growSize) * (growSize + 1) + 1);

  // loop through grid_, looking for 1, then growing them on result
  int row, col;
  for (row = 0; row < gridHeight_; ++row) {
    for (col = 0; col < gridWidth_; ++col) {
      if (grid_[row][col] == 1) {
        getNeighborhood(GridCell(col, row), growSize, neighborhood);
        for (const auto &neighbor : neighborhood) {
          result.set(neighbor, 1);
        }
        neighborhood.clear();
      }
    }
  }
  return result;
}


/**
 * Basic wave propagation. 8-cell, square neighborhoods with wave values differing by 1
 *
 * @param goal
 * @param start
 * @param orthoDist
 * @param diagDist
 * @return
 */

std::pair<GridCell, int> OccGrid::propWavesBasic(GridCell &goal, GridCell &start, int orthoDist, int diagDist) {
  std::queue<GridCell> waveQ;
  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);

  // set goal cell value to 2
  set(goal, 2);
  waveQ.push(goal);

  GridCell currCell;

  // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
  // goal is unreachable from start
  int numCellsVisited = 0;
  while (!waveQ.empty()) {
    currCell = waveQ.front();

    // find free neighbors, update weights, calculate costs, push onto queue
    getNeighborhood(currCell, 1, neighborhood);
    for (auto &neighbor : neighborhood) {
      if (get(neighbor) == 0) {
        setWeight(neighbor, orthoDist, diagDist);
        waveQ.emplace(neighbor);
        ++numCellsVisited;

        // once we've enqueued the start cell, we're done
        if (start.equals(neighbor)) {
          return std::make_pair(neighbor, numCellsVisited);
        }
      }
    }
    neighborhood.clear();

//    std::cout << "q size: " << waveQ.size() << std::endl;

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
 * @param orthoDist - cost to travel one cell orthogonally
 * @param diagDist - cost to travel one cell diagonally
 * @return pair<GridCell, int> -
 *         <cell where waves stopped, number cells placed in processing queue>
 *         if successful
 */
std::pair<GridCell, int> OccGrid::propOFWF(GridCell &goal, GridCell &start, int orthoDist, int diagDist) {
  std::priority_queue<std::pair<int, GridCell>,
                      std::vector<std::pair<int, GridCell>>,
                      CompareGreater>
      waveQ;

  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);

  // set goal cell value to 2 and push to waveQ
  set(goal, 2);
  waveQ.emplace(2, goal);

  GridCell currCell;
  int dx, dy;
  int cost, heuristic;

  // if waveQ.empty(), wave has stopped propagating before the start cell was reached.
  // goal is unreachable from start
  int numCellsVisited {0};
  while (!waveQ.empty()) {
    currCell = waveQ.top().second;

//    std::cout << currCell.toString() << ": " << waveQ.top().first << std::endl;

    // find free neighbors, update weights, calculate costs, push onto priority queue
    getNeighborhood(currCell, 1, neighborhood);
    for (auto &neighbor : neighborhood) {
      if (get(neighbor) == 0) {
        setWeight(neighbor, orthoDist, diagDist);
        dx = std::abs(neighbor.getCol() - start.getCol());
        dy = std::abs(neighbor.getRow() - start.getRow());

//        heuristic = 0.99 * orthoDist * ((dx + dy) + (sqrt(2.0) - 2) * std::min(dx, dy));


        heuristic = orthoDist * (dx + dy) + (diagDist - (2 * orthoDist)) * std::min(dx, dy);
//        heuristic = orthoDist * static_cast<int>(euclideanDist(neighbor, start));

        cost = get(neighbor) + heuristic;
//        std::cout << "  (" << neighbor.getCol()
//                  << "," << neighbor.getRow()
//                  << "): " << cost << "   "
//                  << get(neighbor) << std::endl;

        // Use cost to set priority of neighbor.
        waveQ.emplace(cost, neighbor);
        ++numCellsVisited;

        // once we've enqueued the start cell, we're done
        if (start.equals(neighbor)) {
          return std::make_pair(neighbor, numCellsVisited);
        }
      }
    }
    neighborhood.clear();
//    std::cout << "q size: " << waveQ.size() << std::endl;
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
void OccGrid::setWeight(GridCell &cell, int orthoDist, int diagDist) {
  // find neighbor with minimum weight
  GridCell minNeighbor;
  int minSoFar = INT_MAX;
  int currWeight;

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
      minSoFar = distCell.first;
      minCellSoFar = distCell.second;
    }
  }
  return minCellSoFar;
}


std::pair<int, GridCell> OccGrid::findFree(GridCell &cell, const std::string &direction) {
  int stepCounter = 0;
  int currCol {cell.getCol()};
  int currRow {cell.getRow()};

  while (get(currCol, currRow) != 0) {
    if (direction == "north") {
      --currRow;
    } else if (direction == "south") {
      ++currRow;
    } else if (direction == "west") {
      --currCol;
    } else if (direction == "east") {
      ++currCol;
    }
    ++stepCounter;

    if ((currRow < 0) || (currRow == gridHeight_) || (currCol < 0) || (currCol == gridWidth_)) {
      stepCounter = INT_MAX;
      break;
    }
  }
  std::pair<int, GridCell> freeCell = std::make_pair(stepCounter, GridCell(currCol, currRow));
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
  int row {cell.getRow()};
  int col {cell.getCol()};
  if (row < 0 || row >= gridHeight_ || col < 0 || col >= gridWidth_) {
    std::cout << "Grid index out of bounds. Using closest border cell" << std::endl;
    if (row < 0) {
      cell.setRow(0);
    }
    if (row >= gridHeight_) {
      cell.setRow(gridHeight_ - 1);
    }
    if (col < 0) {
      cell.setCol(0);
    }
    if (col >= gridWidth_) {
      cell.setCol(gridWidth_ - 1);
    }
  }
}


/**
 * Finds the maximum value in a grid.
 *
 * @return maximum value
 */
int OccGrid::findMaxDistance() const {
  std::vector<int> rowMaxValues;
  rowMaxValues.reserve(gridHeight_);
  for (auto &row : grid_) {
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
                              int layers,
                              std::vector<GridCell> &neighborhood) {
  int col = cell.getCol();
  int row = cell.getRow();

  int minRow = std::max(row - layers, 0);
  int maxRow = std::min(row + layers, gridHeight_ - 1);
  int minCol = std::max(col - layers, 0);
  int maxCol = std::min(col + layers, gridWidth_ - 1);
  for (int tempRow = minRow; tempRow <= maxRow; ++tempRow) {
    for (int tempCol = minCol; tempCol <= maxCol; ++tempCol) {
      neighborhood.emplace_back(GridCell(tempCol, tempRow));
    }
  }
  if ((cell.getRow() + cell.getCol()) % 2 == 1) {
    std::reverse(neighborhood.begin(), neighborhood.end());
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
  const int c0 {cell0.getCol()};
  const int c1 {cell1.getCol()};
  const int r0 {cell0.getRow()};
  const int r1 {cell1.getRow()};
  const int dCol {std::abs(c1 - c0)};
  const int dRow {std::abs(r1 - r0)};

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
std::vector<GridCell> OccGrid::drawLineLow(const int c0, const int r0, const int c1, const int r1) {
  int dCol = c1 - c0;
  int dRow = r1 - r0;
  int rowIncrement = 1;
  if (dRow < 0) {
    rowIncrement = -1;
    dRow *= -1;
  }
  int error = 2 * dRow - dCol;
  int currRow = r0;

  std::vector<GridCell> lineOfCells;
  lineOfCells.reserve(c1 - c0 + 1);

  for (int currCol = c0; currCol <= c1; ++currCol) {
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
std::vector<GridCell> OccGrid::drawLineHigh(const int c0, const int r0, const int c1, const int r1) {
  int dCol = c1 - c0;
  int dRow = r1 - r0;
  int colIncrement = 1;
  if (dCol < 0) {
    colIncrement = -1;
    dCol *= -1;
  }
  int error = 2 * dCol - dRow;
  int currCol = c0;

  std::vector<GridCell> lineOfCells;
  lineOfCells.reserve(r1 - r0 + 1);

  for (int currRow = r0; currRow <= r1; ++currRow) {
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
  int dRow = c0.getRow() - c1.getRow();
  int dCol = c0.getCol() - c1.getCol();
  auto dRowSq = static_cast<double>(dRow * dRow);
  auto dColSq = static_cast<double>(dCol * dCol);
  return sqrt(dRowSq + dColSq);
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
  double longitude = gps.getLongitude();
  double latitude = gps.getLatitude();
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
  return metersPerCol * (n + 0.5);
}


double rowToY(int m) {
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

}
