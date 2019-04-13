/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WavefrontNavigator.cpp
 */

#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <iostream>
#include <list>
#include <fstream>
#include <queue>
#include <unordered_map>
#include <climits>
#include <utility>
#include <boost/filesystem.hpp>

#include "../../include/pathplanning/WaveNav.h"



WaveNav::WaveNav() = default;


WaveNav::WaveNav(std::string filename, std::string outName) {
  mapfilename = std::move(filename);
  outPath = std::move(outName);
//  outPath = outName;
  resetNavigator();
//  loadDestinations("../dest_coordinates.txt");
//  loadTestXY("../test_xy.txt");
}


WaveNav::~WaveNav() = default;

void WaveNav::changeOutPath(std::string newOutName) {
  outPath = std::move(newOutName);
  resetNavigator();
}


void WaveNav::resetNavigator() {
//  gridMap.resetGrid();
  gridMap.inputGrid(mapfilename, 5.0);
  gridMap.outputGrid(outPath + "_1-scaled input map.pnm");
  gridMap.growGrid(0.40);

//  debugGrid.resetGrid();
  debugGrid.inputGrid(outPath + "_1-scaled input map.pnm", 1.0);
  debugGrid.convertToDebugGrid();

  wayCells.clear();
  smoothedPath.clear();
}


std::pair<std::vector<GridCell>, double> WaveNav::outputPath() {
  auto it0 = smoothedPath.begin();
  auto it1 = smoothedPath.begin();
  it1++;

  std::vector<GridCell> resultPath;
  double pathLength = 0.0;

  while (it1 != smoothedPath.end()) {
    for (const auto &cell : drawLine(*it0, *it1)) {
      resultPath.emplace_back(cell);
    }
    pathLength += euclideanDist(*it0, *it1);
    it0++;
    it1++;
  }
  return {resultPath, pathLength};
}


WaveNav::PathPlannerOutput WaveNav::planPath(GridCell &start, GridCell &goal, const std::string& waveType) {
  WaveNav::PathPlannerOutput toReturn{};
  toReturn.waveType = waveType;
  resetNavigator();
  gridMap.normCell(start);
  gridMap.normCell(goal);

  auto begin = std::chrono::high_resolution_clock::now();
  auto waveResult = (waveType == "Basic") ?
      gridMap.propWavesBasic(goal, start, 5, 7) :
      gridMap.propOFWF(goal, start, 5, 7);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - begin);
  toReturn.cpuTime = duration.count();

  toReturn.numCellsVisited = waveResult.second;
  toReturn.initialPathLength = static_cast<double>(gridMap.get(start)) / 5;

  if (waveResult.first != goal) {
    calcWayCells(waveResult.first, goal);
    markCells(wayCells, 120);
    debugGrid.outputDebugGrid(outPath + "_2-initialpath.pnm");
    debugGrid.resetGrid();
    debugGrid.inputGrid(outPath + "_1-scaled input map.pnm", 1.0);
    debugGrid.convertToDebugGrid();
    smoothestPath();

    auto pathResults = outputPath();
    toReturn.smoothPathLength = pathResults.second;
    markCells(pathResults.first, 190);
    for (const auto &wayCell : wayCells) {
      markCells(OccGrid::getNeighborhood(wayCell, 1l), 2);
    }
    markCells(wayCells, 2);

    debugGrid.outputDebugGrid(outPath + "_3-smoothpath.pnm");
    return toReturn;
  } else {
    std::cout << "      $ ERROR: cannot find a path to that location.\n" << std::endl;
    return toReturn;
  }
}


void WaveNav::printCells(std::list<GridCell> cells) {
  for (const auto &cell : cells) {
    std::cout << "        " << cell.toString() << std::endl;
  }
}


void WaveNav::markCells(const std::list<GridCell>& cells, long value) {
  for (const auto &cell : cells) {
    debugGrid.set(cell, value);
  }
}

void WaveNav::markCells(const std::vector<GridCell>& cells, long value) {
  for (const auto &cell : cells) {
    debugGrid.set(cell, value);
  }
}


//void WaveNav::loadDestinations(std::string filename) {
//  std::ifstream ifs;
//  std::string name;
//  double latitude, longitude;
//  Point destination;
//
//  ifs.open(filename.c_str(), std::ifstream::in);
//  if (ifs) {
//    while (ifs >> name) {
//      ifs >> latitude >> longitude;
//      destination = Point(longitudeToX(longitude), latitudeToY(latitude));
//      destinations.insert({name, destination});
//    }
//    ifs.close();
//  }
//}
//
//
//void WaveNav::loadTestXY(std::string filename) {
//  std::ifstream ifs;
//  std::string name;
//  double x, y;
//  Point destination;
//
//  ifs.open(filename.c_str(), std::ifstream::in);
//  if (ifs) {
//    while (ifs >> name) {
//      ifs >> x >> y;
//      destination = Point(x, y);
//      testXY.insert({name, destination});
//    }
//    ifs.close();
//  }
//}


void WaveNav::calcWayCells(const GridCell &start, const GridCell &goal) {
  GridCell nextCell;
  for (GridCell curr = start; curr != goal; curr = nextCell) {
    wayCells.emplace_back(curr);
    nextCell = findNextCell(curr);
  }
  wayCells.emplace_back(goal);
//  std::cout << "        Wavefront path found\n";
}


GridCell WaveNav::findNextCell(const GridCell &curr) {
  long currWeight = gridMap.get(curr);

  // curr is the goal cell or is part of an obstacle
  if (currWeight < 3) {
    return curr;
  }

  // find neighbor with minimum weight
  GridCell minCell;
  long minSoFar = LONG_MAX;
  for (const auto &neighbor : gridMap.getNeighborhood(curr, 1)) {
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


void WaveNav::smoothestPath() {
  std::list<GridCell> oldWayCells, newWayCells;
  do {
    oldWayCells = wayCells;
    smoothPath();
    newWayCells = wayCells;
  } while (oldWayCells != newWayCells);
//  std::cout << "\n    *%$ Smoothed path:\n";
//  printCells(wayCells);
  markCells(wayCells, 2);
}


void WaveNav::smoothPath() {
//  std::cout << "\n        Smoothing the path\n";

  long editCount = 1;
//  long beforeSize, afterSize;
  while (editCount > 0) {
//    beforeSize = static_cast<long>(wayCells.size());
//    editCount = smoothPathHelperReverse() + smoothPathHelper();
    editCount = smoothPathHelper2();
//    afterSize = static_cast<long>(wayCells.size());
//    std::cout << "        " << beforeSize << " -> " << afterSize << " cells ("
//              << editCount << " trimmed)" << std::endl;
  }
//  std::cout << "\n    *%$ Smoothed path:\n";
//  printCells(wayCells);
//  markCells(wayCells, 2);
}


long WaveNav::smoothPathHelper() {
  if (wayCells.size() < 3) {
    return 0;
  }

  long modificationCount {0};
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


long WaveNav::smoothPathHelper2() {
  if (wayCells.size() < 3) {
    return 0;
  }

  long modificationCount {0};
  auto it0 = wayCells.begin();
  auto it1 = wayCells.begin();
  auto it2 = wayCells.begin();
  it1++;
  it2++;
  it2++;

  while ((it0 != wayCells.end()) && (it1 != wayCells.end()) && (it2 != wayCells.end())) {
    if (isInLine(*it0, *it2)) {
      it1 = wayCells.erase(it1);
      modificationCount++;
      it0++;
      it1++;
      it2++;
      it2++;
    } else {
      it0++;
      it0++;
      it1++;
      it1++;
      it2++;
      it2++;
    }
  }
  return modificationCount;
}


long WaveNav::smoothPathHelperReverse() {
  if (wayCells.size() < 3) {
    return 0;
  }

  long modificationCount {0};
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
bool WaveNav::isInLine(const GridCell &c0, const GridCell &c1) {
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
std::vector<GridCell> WaveNav::drawLine(const GridCell &c0, const GridCell &c1) {
  long col0 {c0.getCol()};
  long col1 {c1.getCol()};
  long row0 {c0.getRow()};
  long row1 {c1.getRow()};
  long dCol {llabs(col1 - col0)};
  long dRow {llabs(row1 - row0)};

  long col_increment {0};
  if (col1 > col0) {
    col_increment = 1;
  } else if (col1 < col0) {
    col_increment = -1;
  }

  long row_increment {0};
  if (row1 > row0) {
    row_increment = 1;
  } else if (row1 < row0) {
    row_increment = -1;
  }

  long error {dCol - dRow};
  long colAdjustment {dCol * 2};
  long rowAdjustment {dRow * 2};
  long currRow {row0};
  long currCol {col0};
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
  return lineOfCells;
}


double WaveNav::euclideanDist(const GridCell &c0, const GridCell &c1) {
  long dRow = c0.getRow() - c1.getRow();
  long dCol = c0.getCol() - c1.getCol();
  auto dRowSq = static_cast<double>(dRow * dRow);
  auto dColSq = static_cast<double>(dCol * dCol);
  return sqrt(dRowSq + dColSq);
}


WaveNav::PathPlannerOutput findAnyPath(WaveNav &myNav,
                                       const std::string &waveOption,
                                       Point &startC,
                                       Point &goalC) {
  GridCell start = pointToCell(startC);
  GridCell goal = pointToCell(goalC);
//  std::cout << "\n--+*%$ Finding Path between " << start.toString() << " and " << goal.toString() << std::endl;
  return myNav.planPath(start, goal, waveOption);
}


void printOutput(const WaveNav::PathPlannerOutput& out) {
  std::cout << out.waveType << ": \n    cells visited = " << out.numCellsVisited
            << "\n    initial path length = " << out.initialPathLength
            << "\n    smoothed path length = " << out.smoothPathLength
            << "\n    CPU time = " << out.cpuTime << std::endl;
}


int main() {
//  Point *test2p1 = new Point{6, 6};
//  Point test2p2{175, 100};

  boost::filesystem::path p{"output/"};
  boost::filesystem::create_directory(p);


  Point topLeft {8, 6};
  Point bottomRight {185, 183};
  Point bottomLeft {12, 175};
  Point midRight {180, 90};
  Point midLeft {10, 90};

//  std::cout << "\ntest1";
//  WaveNav myNav1("../bitmaps/2dmap-test1.pnm", "test1Basic");
//  auto path1b = findAnyPath(myNav1, "Basic", topLeft, bottomRight);
//  printOutput(path1b);
//  myNav1.changeOutPath("test1OFWF");
//  auto path1o = findAnyPath(myNav1, "OFWF", topLeft, bottomRight);
//  printOutput(path1o);

//  std::cout << "\ntest2\n";
//  WaveNav myNav2("../bitmaps/2dmap-test2.pnm", "test2OFWF");
//  WaveNav::PathPlannerOutput path2o = findAnyPath(myNav2, "OFWF", topLeft, bottomRight);
//  printOutput(path2o);
//  myNav2.changeOutPath("test2Basic");
//  WaveNav::PathPlannerOutput path2b = findAnyPath(myNav2, "Basic", topLeft, bottomRight);
//  printOutput(path2b);
//
//  std::cout << "\ntest2_2\n";
//  myNav2.changeOutPath("test2OFWF_2");
//  WaveNav::PathPlannerOutput path2c = findAnyPath(myNav2, "OFWF", topLeft, test2p2);
//  printOutput(path2c);
//  myNav2.changeOutPath("test2Basic_2");
//  WaveNav::PathPlannerOutput path2d = findAnyPath(myNav2, "Basic", topLeft, test2p2);
//  printOutput(path2d);
//
//
//  std::cout << "\ntest3\n";
//  WaveNav myNav3("../bitmaps/2dmap-test3.pnm", "test3OFWF");
//  WaveNav::PathPlannerOutput path3o = findAnyPath(myNav3, "OFWF", topLeft, bottomRight);
//  printOutput(path3o);
//  myNav3.changeOutPath("test3Basic");
//  WaveNav::PathPlannerOutput path3b = findAnyPath(myNav3, "Basic", topLeft, bottomRight);
//  printOutput(path3b);
//  myNav3.~WaveNav();

//  std::cout << "\ntest4\n";
//  WaveNav myNav4("../bitmaps/2dmap-test4.pnm", "test4OFWF");
//  WaveNav::PathPlannerOutput path4o = findAnyPath(myNav4, "OFWF", topLeft, bottomRight);
//  printOutput(path4o);
//  myNav4.changeOutPath("test4Basic");
//  WaveNav::PathPlannerOutput path4b = findAnyPath(myNav4, "Basic", topLeft, bottomRight);
//  printOutput(path4b);
//  myNav4.~WaveNav();

//  try {
//    std::cout << "\ntest1\n";
//    WaveNav myNav1("../bitmaps/2dmap-test1.pnm", "test1.1_OFWF");
//    WaveNav::PathPlannerOutput path11o = findAnyPath(myNav1, "OFWF", topLeft, bottomRight);
//    printOutput(path11o);
//    myNav1.changeOutPath("test1.1_Basic");
//    WaveNav::PathPlannerOutput path11b = findAnyPath(myNav1, "Basic", topLeft, bottomRight);
//    printOutput(path11b);
//    myNav1.changeOutPath("test1.2_OFWF");
//    WaveNav::PathPlannerOutput path12o = findAnyPath(myNav1, "OFWF", bottomLeft, midRight);
//    printOutput(path12o);
//    myNav1.changeOutPath("test1.2_Basic");
//    WaveNav::PathPlannerOutput path12b = findAnyPath(myNav1, "Basic", bottomLeft, midRight);
//    printOutput(path12b);
//  } catch (std::runtime_error& e) {
//    std::cout << e.what() << std::endl;
//  }
//
//
  try {
    std::cout << "\ntest2\n";
    WaveNav myNav2("../bitmaps/2dmap-test2.pnm", "test2.1_OFWF");
    WaveNav::PathPlannerOutput path21o = findAnyPath(myNav2, "OFWF", topLeft, bottomRight);
    printOutput(path21o);
    myNav2.changeOutPath("test2.1_Basic");
    WaveNav::PathPlannerOutput path21b = findAnyPath(myNav2, "Basic", topLeft, bottomRight);
    printOutput(path21b);
    myNav2.changeOutPath("test2.2_OFWF");
    WaveNav::PathPlannerOutput path22o = findAnyPath(myNav2, "OFWF", midLeft, midRight);
    printOutput(path22o);
    myNav2.changeOutPath("test2.2_Basic");
    WaveNav::PathPlannerOutput path22b = findAnyPath(myNav2, "Basic", midLeft, midRight);
    printOutput(path22b);
    myNav2.changeOutPath("test2.3_OFWF");
    WaveNav::PathPlannerOutput path23o = findAnyPath(myNav2, "OFWF", midRight, midLeft);
    printOutput(path22o);
    myNav2.changeOutPath("test2.3_Basic");
    WaveNav::PathPlannerOutput path23b = findAnyPath(myNav2, "Basic", midRight, midLeft);
    printOutput(path22b);
  } catch (std::runtime_error &e) {
    std::cout << e.what() << std::endl;
  }
//
//  try {
//    std::cout << "\ntest3\n";
//    WaveNav myNav3("../bitmaps/2dmap-test3.pnm", "test3.1_OFWF");
//    WaveNav::PathPlannerOutput path31o = findAnyPath(myNav3, "OFWF", topLeft, bottomRight);
//    printOutput(path31o);
//    myNav3.changeOutPath("test3.1_Basic");
//    WaveNav::PathPlannerOutput path31b = findAnyPath(myNav3, "Basic", topLeft, bottomRight);
//    printOutput(path31b);
//    myNav3.changeOutPath("test3.2_OFWF");
//    WaveNav::PathPlannerOutput path32o = findAnyPath(myNav3, "OFWF", bottomLeft, midRight);
//    printOutput(path32o);
//    myNav3.changeOutPath("test3.2_Basic");
//    WaveNav::PathPlannerOutput path32b = findAnyPath(myNav3, "Basic", bottomLeft, midRight);
//    printOutput(path32b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }

//  try {
//    std::cout << "\ntest4\n";
//    WaveNav myNav4("../bitmaps/2dmap-test4.pnm", "test4.1_OFWF");
//    WaveNav::PathPlannerOutput path41o = findAnyPath(myNav4, "OFWF", topLeft, bottomRight);
//    printOutput(path41o);
//    myNav4.changeOutPath("test4.1_Basic");
//    WaveNav::PathPlannerOutput path41b = findAnyPath(myNav4, "Basic", topLeft, bottomRight);
//    printOutput(path41b);
//    myNav4.changeOutPath("test4.2_OFWF");
//    WaveNav::PathPlannerOutput path42o = findAnyPath(myNav4, "OFWF", midRight, midLeft);
//    printOutput(path42o);
//    myNav4.changeOutPath("test4.2_Basic");
//    WaveNav::PathPlannerOutput path42b = findAnyPath(myNav4, "Basic", midRight, midLeft);
//    printOutput(path42b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }
//
//  try {
//    std::cout << "\ntest5\n";
//    WaveNav myNav5("../bitmaps/2dmap-test5.pnm", "test5.1_OFWF");
//    WaveNav::PathPlannerOutput path51o = findAnyPath(myNav5, "OFWF", topLeft, bottomRight);
//    printOutput(path51o);
//    myNav5.changeOutPath("test5.1_Basic");
//    WaveNav::PathPlannerOutput path51b = findAnyPath(myNav5, "Basic", topLeft, bottomRight);
//    printOutput(path51b);
//    myNav5.changeOutPath("test5.2_OFWF");
//    WaveNav::PathPlannerOutput path52o = findAnyPath(myNav5, "OFWF", bottomLeft, midRight);
//    printOutput(path52o);
//    myNav5.changeOutPath("test5.2_Basic");
//    WaveNav::PathPlannerOutput path52b = findAnyPath(myNav5, "Basic", bottomLeft, midRight);
//    printOutput(path52b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }
//
//  try {
//    std::cout << "\ntest6\n";
//    WaveNav myNav6("../bitmaps/2dmap-test6.pnm", "test6.1_OFWF");
//    WaveNav::PathPlannerOutput path61o = findAnyPath(myNav6, "OFWF", topLeft, bottomRight);
//    printOutput(path61o);
//    myNav6.changeOutPath("test6.1_Basic");
//    WaveNav::PathPlannerOutput path61b = findAnyPath(myNav6, "Basic", topLeft, bottomRight);
//    printOutput(path61b);
//    myNav6.changeOutPath("test6.2_OFWF");
//    WaveNav::PathPlannerOutput path62o = findAnyPath(myNav6, "OFWF", bottomLeft, midRight);
//    printOutput(path62o);
//    myNav6.changeOutPath("test6.2_Basic");
//    WaveNav::PathPlannerOutput path62b = findAnyPath(myNav6, "Basic", bottomLeft, midRight);
//    printOutput(path62b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }

//  try {
//    std::cout << "\ntest 2d map01\n";
//    WaveNav myNav1("../bitmaps/2dmap-01.pnm", "2dmap1.1_OFWF");
//    WaveNav::PathPlannerOutput path11o = findAnyPath(myNav1, "OFWF", topLeft, bottomRight);
//    printOutput(path11o);
//    myNav1.changeOutPath("2dmap1.1_Basic");
//    WaveNav::PathPlannerOutput path11b = findAnyPath(myNav1, "Basic", topLeft, bottomRight);
//    printOutput(path11b);
//    myNav1.changeOutPath("2dmap1.2_OFWF");
//    WaveNav::PathPlannerOutput path12o = findAnyPath(myNav1, "OFWF", bottomLeft, midRight);
//    printOutput(path12o);
//    myNav1.changeOutPath("2dmap1.2_Basic");
//    WaveNav::PathPlannerOutput path12b = findAnyPath(myNav1, "Basic", bottomLeft, midRight);
//    printOutput(path12b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }

}




