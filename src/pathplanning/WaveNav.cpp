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
#include "../../include/pathplanning/OccGrid.h"


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


std::pair<std::vector<GridCell>, double> WaveNav::drawSmoothedPath() {
  auto it0 = smoothedPath.begin();
  auto it1 = smoothedPath.begin();
  it1++;

  std::vector<GridCell> resultPath;
  double pathLength = 0.0;

  while (it1 != smoothedPath.end()) {
    for (const auto &cell : OccGrid::drawLine(*it0, *it1)) {
      markCell(cell, 190);
    }
    pathLength += OccGrid::euclideanDist(*it0, *it1);
    it0++;
    it1++;
  }
  return std::make_pair(resultPath, pathLength);
}


WaveNav::PathPlannerOutput WaveNav::planPath(GridCell &start, GridCell &goal, const std::string& waveType) {
  WaveNav::PathPlannerOutput toReturn{};
  toReturn.waveType = waveType;
  resetNavigator();
  gridMap.normCell(start);
  gridMap.normCell(goal);

  auto begin = std::chrono::high_resolution_clock::now();
  auto waveResult = (waveType == "Basic") ?
      gridMap.propWavesBasic(goal, start, 500, 707) :
      gridMap.propOFWF(goal, start, 500, 707);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - begin);
  toReturn.cpuTime = duration.count();

  GridCell finalCell = waveResult.first;
  toReturn.numCellsVisited = waveResult.second;
  toReturn.initialPathLength = static_cast<double>(gridMap.get(finalCell)) / 500;


  if (OccGrid::isNear(finalCell, start)) {
    calcWayCells(finalCell, goal);
    markCells(wayCells, 120);
    debugGrid.outputDebugGrid(outPath + "_2-initialpath.pnm");
    debugGrid.resetGrid();
    debugGrid.inputGrid(outPath + "_1-scaled input map.pnm", 1.0);
    debugGrid.convertToDebugGrid();

    gridMap.resetGrid();
    gridMap.inputGrid(outPath + "_1-scaled input map.pnm", 1.0);
    smoothPath();

    auto pathResults = drawSmoothedPath();
    toReturn.smoothPathLength = pathResults.second;
    markCells(pathResults.first, 190);
    for (const auto &cell : smoothedPath) {
      markCells(OccGrid::getNeighborhood(cell, 1l), 2);
    }

    debugGrid.outputDebugGrid(outPath + "_3-smoothpath.pnm");
    return toReturn;
  } else {
    std::cout << "      $ ERROR: cannot find a path to that location.\n" << std::endl;
    return toReturn;
  }
}


void WaveNav::printCells(const std::list<GridCell>& cells) {
  for (const auto &cell : cells) {
    std::cout << "        " << cell.toString() << std::endl;
  }
}

void WaveNav::printCells(const std::vector<GridCell>& cells) {
  for (const auto &cell : cells) {
    std::cout << "        " << cell.toString() << std::endl;
  }
}

void WaveNav::markCell(const GridCell & cell, long value) {
  debugGrid.set(cell, value);
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
  for (const auto &neighbor : OccGrid::getNeighborhood(curr, 1)) {
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


/**
 * Path simplification which solves the following: given a sequence of cells (c1, ... , cn) representing a path between
 * start and goal cells, the solution will satisfy:
 *    (1) has same start and goal cells
 *    (2) there is a clear line of sight between consecutive solution cells
 *    (3) total path length is minimized
 *
 */
void WaveNav::smoothPath() {
  smoothedPath.assign(wayCells.begin(), wayCells.end());
  int oldNodeCount, newNodeCount, editCount;
  do {
    oldNodeCount = smoothedPath.size();
    smoothPathHelper();
    newNodeCount = smoothedPath.size();
    editCount = oldNodeCount - newNodeCount;
  } while (editCount != 0);
}


long WaveNav::smoothPathHelper() {
  if (smoothedPath.size() < 3) {
    return 0;
  }

  long modificationCount {0};
  auto it0 = smoothedPath.begin();
  auto it2 = smoothedPath.begin();
  auto it_end = smoothedPath.end();
//  it2++;
//  it2++;

  while ((it0 != it_end) && (it2 != it_end)) {
    if (smoothedPath.size() < 3) {
      break;
    }

    it2++;
    it2++;
    while ((it2 != it_end) && (gridMap.isInLine(*it0, *it2))) {
      it2++;
      modificationCount++;
    }
    modificationCount--;
    it2--;
    it0++;
    it0 = smoothedPath.erase(it0, it2);
  }
  return modificationCount;
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

//  GridCell c1 = GridCell(10, 15);
//  GridCell c0 = GridCell(10, 70);
//
//  WaveNav::printCells(OccGrid::drawLine(c0, c1));


  boost::filesystem::path p{"output/"};
  boost::filesystem::create_directory(p);


  Point topLeft {8, 6};
  Point bottomRight {185, 183};
  Point bottomLeft {12, 175};
  Point midRight {180, 90};
  Point midLeft {10, 90};

//  try {
//    std::cout << "\ntest1\n";
//    WaveNav myNav1("../bitmaps/2dmap-test1.pnm", "output/test1.1_OFWF");
//    WaveNav::PathPlannerOutput path11o = findAnyPath(myNav1, "OFWF", topLeft, bottomRight);
//    printOutput(path11o);
//    myNav1.changeOutPath("output/test1.1_Basic");
//    WaveNav::PathPlannerOutput path11b = findAnyPath(myNav1, "Basic", topLeft, bottomRight);
//    printOutput(path11b);
//    myNav1.changeOutPath("output/test1.2_OFWF");
//    WaveNav::PathPlannerOutput path12o = findAnyPath(myNav1, "OFWF", bottomLeft, midRight);
//    printOutput(path12o);
//    myNav1.changeOutPath("output/test1.2_Basic");
//    WaveNav::PathPlannerOutput path12b = findAnyPath(myNav1, "Basic", bottomLeft, midRight);
//    printOutput(path12b);
//  } catch (std::runtime_error& e) {
//    std::cout << e.what() << std::endl;
//  }
//
//
//  try {
//    std::cout << "\ntest2\n";
//    WaveNav myNav2("../bitmaps/2dmap-test2.pnm", "output/test2.1_OFWF");
//    WaveNav::PathPlannerOutput path21o = findAnyPath(myNav2, "OFWF", topLeft, bottomRight);
//    printOutput(path21o);
//    myNav2.changeOutPath("output/test2.1_Basic");
//    WaveNav::PathPlannerOutput path21b = findAnyPath(myNav2, "Basic", topLeft, bottomRight);
//    printOutput(path21b);
//    myNav2.changeOutPath("output/test2.2_OFWF");
//    WaveNav::PathPlannerOutput path22o = findAnyPath(myNav2, "OFWF", midLeft, midRight);
//    printOutput(path22o);
//    myNav2.changeOutPath("output/test2.2_Basic");
//    WaveNav::PathPlannerOutput path22b = findAnyPath(myNav2, "Basic", midLeft, midRight);
//    printOutput(path22b);
//    myNav2.changeOutPath("output/test2.3_OFWF");
//    WaveNav::PathPlannerOutput path23o = findAnyPath(myNav2, "OFWF", midRight, midLeft);
//    printOutput(path22o);
//    myNav2.changeOutPath("output/test2.3_Basic");
//    WaveNav::PathPlannerOutput path23b = findAnyPath(myNav2, "Basic", midRight, midLeft);
//    printOutput(path22b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }
//
//  try {
//    std::cout << "\ntest3\n";
//    WaveNav myNav3("../bitmaps/2dmap-test3.pnm", "output/test3.1_OFWF");
//    WaveNav::PathPlannerOutput path31o = findAnyPath(myNav3, "OFWF", topLeft, bottomRight);
//    printOutput(path31o);
//    myNav3.changeOutPath("output/test3.1_Basic");
//    WaveNav::PathPlannerOutput path31b = findAnyPath(myNav3, "Basic", topLeft, bottomRight);
//    printOutput(path31b);
//    myNav3.changeOutPath("output/test3.2_OFWF");
//    WaveNav::PathPlannerOutput path32o = findAnyPath(myNav3, "OFWF", bottomLeft, midRight);
//    printOutput(path32o);
//    myNav3.changeOutPath("output/test3.2_Basic");
//    WaveNav::PathPlannerOutput path32b = findAnyPath(myNav3, "Basic", bottomLeft, midRight);
//    printOutput(path32b);
//  } catch (std::runtime_error &e) {
//    std::cout << e.what() << std::endl;
//  }

  try {
    std::cout << "\ntest4\n";
    WaveNav myNav4("../bitmaps/2dmap-test4.pnm", "output/test4.1_OFWF");
    WaveNav::PathPlannerOutput path41o = findAnyPath(myNav4, "OFWF", topLeft, bottomRight);
    printOutput(path41o);
    myNav4.changeOutPath("output/test4.1_Basic");
    WaveNav::PathPlannerOutput path41b = findAnyPath(myNav4, "Basic", topLeft, bottomRight);
    printOutput(path41b);
    myNav4.changeOutPath("output/test4.2_OFWF");
    WaveNav::PathPlannerOutput path42o = findAnyPath(myNav4, "OFWF", midRight, midLeft);
    printOutput(path42o);
    myNav4.changeOutPath("output/test4.2_Basic");
    WaveNav::PathPlannerOutput path42b = findAnyPath(myNav4, "Basic", midRight, midLeft);
    printOutput(path42b);
  } catch (std::runtime_error &e) {
    std::cout << e.what() << std::endl;
  }

  try {
    std::cout << "\ntest5\n";
    WaveNav myNav5("../bitmaps/2dmap-test5.pnm", "output/test5.1_OFWF");
    WaveNav::PathPlannerOutput path51o = findAnyPath(myNav5, "OFWF", topLeft, bottomRight);
    printOutput(path51o);
    myNav5.changeOutPath("output/test5.1_Basic");
    WaveNav::PathPlannerOutput path51b = findAnyPath(myNav5, "Basic", topLeft, bottomRight);
    printOutput(path51b);
    myNav5.changeOutPath("output/test5.2_OFWF");
    WaveNav::PathPlannerOutput path52o = findAnyPath(myNav5, "OFWF", bottomLeft, midRight);
    printOutput(path52o);
    myNav5.changeOutPath("output/test5.2_Basic");
    WaveNav::PathPlannerOutput path52b = findAnyPath(myNav5, "Basic", bottomLeft, midRight);
    printOutput(path52b);
  } catch (std::runtime_error &e) {
    std::cout << e.what() << std::endl;
  }

  try {
    std::cout << "\ntest6\n";
    WaveNav myNav6("../bitmaps/2dmap-test6.pnm", "output/test6.1_OFWF");
    WaveNav::PathPlannerOutput path61o = findAnyPath(myNav6, "OFWF", topLeft, bottomRight);
    printOutput(path61o);
    myNav6.changeOutPath("output/test6.1_Basic");
    WaveNav::PathPlannerOutput path61b = findAnyPath(myNav6, "Basic", topLeft, bottomRight);
    printOutput(path61b);
    myNav6.changeOutPath("output/test6.2_OFWF");
    WaveNav::PathPlannerOutput path62o = findAnyPath(myNav6, "OFWF", bottomLeft, midRight);
    printOutput(path62o);
    myNav6.changeOutPath("output/test6.2_Basic");
    WaveNav::PathPlannerOutput path62b = findAnyPath(myNav6, "Basic", bottomLeft, midRight);
    printOutput(path62b);
  } catch (std::runtime_error &e) {
    std::cout << e.what() << std::endl;
  }



}




