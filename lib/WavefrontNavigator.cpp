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

typedef std::list<GridCell>::iterator list_it;

WavefrontNavigator::WavefrontNavigator() = default;


WavefrontNavigator::WavefrontNavigator(const std::string filename) {
  mapfilename = filename;
  resetNavigator();
  loadDestinations("../dest_coordinates.txt");
  loadTestXY("../test_xy.txt");
}


WavefrontNavigator::~WavefrontNavigator() = default;


void WavefrontNavigator::resetNavigator() {
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


std::vector<GridCell> WavefrontNavigator::outputPath() {
  auto it0 {wayCells.begin()};
  auto it1 {wayCells.begin()};
  it1++;

  std::vector<GridCell> resultPath;

  while (it1 != wayCells.end()) {
    for (const auto &cell : drawLine(*it0, *it1)) {
      resultPath.emplace_back(cell);
    }
    it0++;
    it1++;
  }
  return resultPath;
}


bool WavefrontNavigator::planPath(GridCell start, GridCell goal, const std::string waveType) {
  bool pathFound {false};
  resetNavigator();
  gridMap.normCell(start);
  gridMap.normCell(goal);

  GridCell waveResult {};
  if (waveType == "Basic") {
    waveResult = gridMap.propWavesBasic(goal, start);
  } else if (waveType == "OFWF") {
    waveResult = gridMap.propOFWF(goal, start);
  }

  if (waveResult != goal) {
    calcWayCells(waveResult, goal);
    smoothPath();

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
  for (const auto &cell : cells) {
    std::cout << "        " << cell.toString() << std::endl;
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
      destination = Point(longitudeToX(longitude), latitudeToY(latitude));
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
  for (GridCell curr = start; curr != goal; curr = nextCell) {
    wayCells.emplace_back(curr);
    debugGrid.set(curr, 1);
    nextCell = findNextCell(curr);
  }
  wayCells.emplace_back(goal);
  std::cout << "        Wavefront path found\n";
  debugGrid.outputGrid("../debugOutput/3-initialPath.pnm");
  debugGrid.inputGrid("../debugOutput/1-scaled input map.pnm", 1.0);
}


GridCell WavefrontNavigator::findNextCell(const GridCell &curr) {
  int currWeight {gridMap.get(curr)};

  // curr is the goal cell or is part of an obstacle
  if (currWeight < 3) {
    return curr;
  }

  // find neighbor with minimum weight
  GridCell minCell {curr};
  int minSoFar {currWeight};
  for (const auto &neighbor : gridMap.getNeighbors(curr, 0)) {
    currWeight = gridMap.get(neighbor);
    if ((currWeight < minSoFar) && (currWeight > 1)) {
      minSoFar = currWeight;
      minCell = neighbor;
    }
  }
  if (minCell == curr) {
    std::cout << "      $ ERROR: no neighboring cell has a lower value than current cell." << std::endl;
  }
  return minCell;
}


void WavefrontNavigator::smoothPath() {
  std::cout << "\n        Smoothing the path\n";

  int editCount {1};
  int n0, n1;
  while (editCount > 0) {
    n0 = static_cast<int>(wayCells.size());
    editCount = smoothPathHelper() + smoothPathHelperReverse();
    n1 = static_cast<int>(wayCells.size());
    std::cout << "        " << n0 << " -> " << n1 << " cells ("
              << editCount << " trimmed)" << std::endl;
  }
  std::cout << "\n    *%$ Smoothed path:\n";
  printCells(wayCells);
  markCells(wayCells);
}


int WavefrontNavigator::smoothPathHelper() {
  if (wayCells.size() < 3) {
    return 0;
  }

  int modificationCount {0};
  auto it0 = wayCells.begin();
  auto it1 = wayCells.begin();
  auto it2 = wayCells.begin();
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

  int modificationCount {0};
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
//  std::list<GridCell> lineSegment = drawLine(c0, c1);

  // check if any cell in the list is occupied by an obstacle.
  for (const auto &gc : drawLine(c0, c1)) {
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
std::vector<GridCell> WavefrontNavigator::drawLine(const GridCell &c0, const GridCell &c1) {
  int col0 {c0.getCol()};
  int col1 {c1.getCol()};
  int row0 {c0.getRow()};
  int row1 {c1.getRow()};
  int dCol {abs(col1 - col0)};
  int dRow {abs(row1 - row0)};

  int col_increment {0};
  if (col1 > col0) {
    col_increment = 1;
  } else if (col1 < col0) {
    col_increment = -1;
  }

  int row_increment {0};
  if (row1 > row0) {
    row_increment = 1;
  } else if (row1 < row0) {
    row_increment = -1;
  }

  int error {dCol - dRow};
  int colAdjustment {dCol * 2};
  int rowAdjustment {dRow * 2};
  int currRow {row0};
  int currCol {col0};
  GridCell currCell(currCol, currRow);

  std::vector<GridCell> lineOfCells;
  while (currCell != c1) {
    lineOfCells.emplace_back(currCell);
    if (error > 0) {
      currCol += col_increment;
      error -= rowAdjustment;
    } else if (error < 0) {
      currRow += row_increment;
      error += colAdjustment;
    } else if (error == 0) {
      currCol += col_increment;
      error -= rowAdjustment;
      currRow += row_increment;
      error += colAdjustment;
    }
    currCell = GridCell(currCol, currRow);
  }
  lineOfCells.emplace_back(c1);
//
//  for (int cellCount = 1 + dRow + dCol; cellCount > 0; --cellCount) {
//    lineOfCells.emplace_back(GridCell(currCol, currRow));
//
//    // error value basically tells us if the current cell lies above or below the actual line.
//    // We use this info to find the next cell that is closest to the line.
//    if (error > 0) {
//      currCol += col_increment;
//      error -= dRow;
//    } else if (error < 0) {
//      currRow += row_increment;
//      error += dCol;
//    } else if (error == 0) {
//      currCol += col_increment;
//      error -= dRow;
//      currRow += row_increment;
//      error += dCol;
//      cellCount--;
//    }
//  }
  return lineOfCells;
}


OccGrid *WavefrontNavigator::getGridMap() {
  return &gridMap;
}


OccGrid *WavefrontNavigator::getDebugGrid() {
  return &debugGrid;
}


/**
 * Main program driver
 *
 * @param myNav
 * @param startTitle
 * @param goalTitle
 */

void findPath(WavefrontNavigator &myNav,
              const std::string &waveOption,
              const std::string &startTitle,
              const std::string &goalTitle) {
  Point start, goal;
  try {
    start = myNav.getDestinations().at(startTitle);
    goal = myNav.getDestinations().at(goalTitle);
  } catch (const std::out_of_range &oor) {
    start = myNav.getTestXY().at(startTitle);
    goal = myNav.getTestXY().at(goalTitle);
  }
  GridCell startCell = pointToCell(start);
  GridCell goalCell = pointToCell(goal);
//  std::cout << "\n--+*%$ Finding Path between " << startCell.toString() << " and " << goalCell.toString() << std::endl;
  if (myNav.planPath(startCell, goalCell, waveOption)) {
//    std::cout << "SUCCESS\n";
  }
}


std::list<GridCell> findAnyPath(WavefrontNavigator &myNav, const std::string &waveOption, GPS &startC, GPS &goalC) {
  GridCell start = gpsToCell(startC);
  GridCell goal = gpsToCell(goalC);
//  std::cout << "\n--+*%$ Finding Path between " << start.toString() << " and " << goal.toString() << std::endl;
  if (myNav.planPath(start, goal, waveOption)) {
    return myNav.getWayCells();
  }
}


int main(int argc, char *argv[]) {
  WavefrontNavigator myNav("../bitmaps/2dmap-test4.pnm");
//    findPath(myNav, "OFWF", "dest01", "dest02");
//    findPath(myNav, "OFWF", "dest01", "dest03");
//    findPath(myNav, "OFWF", "dest01", "dest04");
  findPath(myNav, "OFWF", "dest01", "dest08");
  findPath(myNav, "Basic", "dest01", "dest08");
//    findPath(myNav, "OFWF", "dest06", "dest09");
//    findPath(myNav, "OFWF", "dest06", "dest10");
//    findPath(myNav, "OFWF", "dest01", "dest10");
//    findPath(myNav, "Basic", "dest01", "dest10");
  GPS coord1(((MAX_LONG - MIN_LONG) / 2) + MIN_LONG,
             ((MAX_LAT - MIN_LAT) / 2) + MIN_LAT);
  GPS coord2(MIN_LONG + 0.0005, ((MAX_LAT - MIN_LAT) / 2) + MIN_LAT);

  std::list<GridCell> path = findAnyPath(myNav, "OFWF", coord2, coord1);
//  for (auto &cell : path) {
//    std::cout << cell.toString() << std::endl;
//  }
}

