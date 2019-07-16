/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WaveNav.cpp
 */

#include "pathplanning/DebugGrid.h"
#include "pathplanning/OccGrid.h"
#include "pathplanning/WaveNav.h"


WaveNav::WaveNav(std::string filename, std::string outName) {
  mapfilename = std::move(filename);
  outPath = std::move(outName);
  gridMap = OccGrid(mapfilename, SCALE_MAP);
  gridMap.outputGrid(outPath + "_1-scaled input map.pnm");
  gridMap = gridMap.growGrid(0.80);

  debugGrid = DebugGrid(outPath + "_1-scaled input map.pnm", 1.0);

  wayCells.clear();
  smoothedPath.clear();
//  loadDestinations("../dest_coordinates.txt");
//  loadTestXY("../test_xy.txt");
}


WaveNav::~WaveNav() = default;


void WaveNav::initializeNavigator() {
  gridMap = OccGrid(mapfilename, SCALE_MAP);
  gridMap.outputGrid(outPath + "_1-scaled input map.pnm");
  gridMap = gridMap.growGrid(0.80);

  debugGrid = DebugGrid(outPath + "_1-scaled input map.pnm", 1.0);

  wayCells.clear();
  smoothedPath.clear();
}


void WaveNav::changeOutPath(std::string &newOutPath) {
  outPath = newOutPath;
//  initializeNavigator();
}


double WaveNav::markSmoothedPath() {
  auto it0 = smoothedPath.begin();
  auto it1 = smoothedPath.begin();
  it1++;

  double pathLength = 0.0;

  while (it1 != smoothedPath.end()) {
    debugGrid.markCells(OccGrid::drawLine(*it0, *it1), Pixel(0, 0, 255));
    pathLength += OccGrid::euclideanDist(*it0, *it1);
    it0++;
    it1++;
  }
  return pathLength;
}


WaveNav::PathPlannerOutput WaveNav::planPath(GridCell &start, GridCell &goal, const std::string& waveType) {
  WaveNav::PathPlannerOutput toReturn{};
  toReturn.waveType = waveType;
//  initializeNavigator();
  gridMap.normCell(start);
  gridMap.normCell(goal);

  auto begin = std::chrono::high_resolution_clock::now();
  auto waveResult = (waveType == "Basic") ?
      gridMap.propWavesBasic(goal, start, 500, 707) :
      gridMap.propOFWF(goal, start, 500, 707);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - begin);
  toReturn.cpuTime = duration.count();

  GridCell finalCell = waveResult.first;
  toReturn.numCellsVisited = waveResult.second;
  toReturn.initialPathLength = static_cast<double>(gridMap.get(finalCell)) / 500;

  if (waveType == "Basic") {
    debugGrid.markWaves(gridMap, "R");
  } else {
    debugGrid.markWaves(gridMap, "G");
  }

  if (finalCell.equals(start)) {
    calcWayCells(finalCell, goal);
    debugGrid.markCells(wayCells, Pixel(120));

    gridMap = OccGrid(outPath + "_1-scaled input map.pnm", 1.0);
    gridMap = gridMap.growGrid(0.4);

    smoothPath();

    toReturn.smoothPathLength = markSmoothedPath();

    std::vector<GridCell> neighborhood;
    neighborhood.reserve(9);
    for (const auto &cell : smoothedPath) {
      OccGrid::getNeighborhood(cell, 1, neighborhood);
      debugGrid.markCells(neighborhood, Pixel(0, 0, 255));
      neighborhood.clear();
    }

    debugGrid.markStart(start);
    debugGrid.markGoal(goal);

    debugGrid.outputGrid(outPath + "_2-debug.pnm");
    return toReturn;
  } else {
    std::cout << "      $ ERROR: cannot find a path to that location.\n" << std::endl;
    return toReturn;
  }
}


WaveNav::PathPlannerOutput WaveNav::planPath(Point &start, Point &goal, const std::string &waveType) {
  GridCell startCell = pointToCell(start);
  GridCell goalCell = pointToCell(goal);
  return WaveNav::planPath(startCell, goalCell, waveType);
}


WaveNav::PathPlannerOutput WaveNav::planPath(GPS &start, GPS &goal, const std::string &waveType) {
  GridCell startCell = gpsToCell(start);
  GridCell goalCell = gpsToCell(goal);
  return WaveNav::planPath(startCell, goalCell, waveType);
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


void WaveNav::loadDestinations(std::string& filename) {
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
  }
  ifs.close();
}


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

  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);
  OccGrid::getNeighborhood(curr, 1, neighborhood);

  for (const auto &neighbor : neighborhood) {
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
 *    (3) total path length and number of nodes are minimized
 */
void WaveNav::smoothPath() {
  smoothedPath.assign(wayCells.begin(), wayCells.end());
  int pathSizeBeforeSmooth, pathSizeAfterSmooth, smoothModifications;
  do {
    pathSizeBeforeSmooth = smoothedPath.size();
    smoothPathHelper();
    pathSizeAfterSmooth = smoothedPath.size();
    smoothModifications = pathSizeBeforeSmooth - pathSizeAfterSmooth;
  } while (smoothModifications != 0);
}


void WaveNav::smoothPathHelper() {
  if (smoothedPath.size() < 3) {
    return;
  }

  auto it0 = smoothedPath.begin();
  auto it1 = smoothedPath.begin();
  auto it_end = smoothedPath.end();

  while ((it0 != it_end) && (it1 != it_end)) {
    if (smoothedPath.size() < 3) {
      break;
    }
    it1++;
    it1++;
    while ((it1 != it_end) && (gridMap.isInLine(*it0, *it1))) {
      it1++;
    }
    it1--;
    it0++;
    it0 = smoothedPath.erase(it0, it1);
  }
}


WaveNav::PathPlannerOutput findAnyPath(WaveNav& myNav,
                                       const std::string& waveOption,
                                       Point& startC,
                                       Point& goalC) {
  GridCell start = pointToCell(startC);
  GridCell goal = pointToCell(goalC);
  return myNav.planPath(start, goal, waveOption);
}


void printOutput(const WaveNav::PathPlannerOutput& out) {
  std::cout << "  " << out.waveType << ": \n    CPU time = " << out.cpuTime
            << "\n    cells visited = " << out.numCellsVisited
            << "\n    initial path length = " << out.initialPathLength
            << "\n    smoothed path length = " << out.smoothPathLength << std::endl;

}


int main() {

  boost::filesystem::path p{"output/"};
  boost::filesystem::create_directory(p);

  Point topLeft {8, 6};
  Point bottomRight {185, 183};
  Point bottomLeft {12, 175};
  Point midRight {180, 90};
  Point midLeft {10, 90};
  Point centerLeft {70, 65};
  Point middle {90, 85};

  std::cout << "\ntest1\n";
  WaveNav myNav("../bitmaps/2dmap-test1.pnm", "output/test1.1_OFWF");
  WaveNav::PathPlannerOutput path11o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path11o);
  myNav = WaveNav("../bitmaps/2dmap-test1.pnm", "output/test1.1_Basic");
  WaveNav::PathPlannerOutput path11b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path11b);
  myNav = WaveNav("../bitmaps/2dmap-test1.pnm", "output/test1.2_OFWF");
  WaveNav::PathPlannerOutput path12o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
  printOutput(path12o);
  myNav = WaveNav("../bitmaps/2dmap-test1.pnm", "output/test1.2_Basic");
  WaveNav::PathPlannerOutput path12b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
  printOutput(path12b);

  std::cout << "\ntest2\n";
  myNav = WaveNav("../bitmaps/2dmap-test2.pnm", "output/test2.1_OFWF");
  WaveNav::PathPlannerOutput path21o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path21o);
  myNav = WaveNav("../bitmaps/2dmap-test2.pnm", "output/test2.1_Basic");
  WaveNav::PathPlannerOutput path21b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path21b);
  myNav = WaveNav("../bitmaps/2dmap-test2.pnm", "output/test2.2_OFWF");
  WaveNav::PathPlannerOutput path22o = findAnyPath(myNav, "OFWF", midLeft, midRight);
  printOutput(path22o);
  myNav = WaveNav("../bitmaps/2dmap-test2.pnm", "output/test2.2_Basic");
  WaveNav::PathPlannerOutput path22b = findAnyPath(myNav, "Basic", midLeft, midRight);
  printOutput(path22b);
  myNav = WaveNav("../bitmaps/2dmap-test2.pnm", "output/test2.3_OFWF");
  WaveNav::PathPlannerOutput path23o = findAnyPath(myNav, "OFWF", midRight, midLeft);
  printOutput(path23o);
  myNav = WaveNav("../bitmaps/2dmap-test2.pnm", "output/test2.3_Basic");
  WaveNav::PathPlannerOutput path23b = findAnyPath(myNav, "Basic", midRight, midLeft);
  printOutput(path23b);

  Point upperMiddle {60, 30};
  std::cout << "\ntest3\n";
  myNav = WaveNav("../bitmaps/2dmap-test3.pnm", "output/test3.1_OFWF");
  WaveNav::PathPlannerOutput path31o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path31o);
  myNav = WaveNav("../bitmaps/2dmap-test3.pnm", "output/test3.1_Basic");
  WaveNav::PathPlannerOutput path31b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path31b);
  myNav = WaveNav("../bitmaps/2dmap-test3.pnm", "output/test3.2_OFWF");
  WaveNav::PathPlannerOutput path32o = findAnyPath(myNav, "OFWF", topLeft, upperMiddle);
  printOutput(path32o);
  myNav = WaveNav("../bitmaps/2dmap-test3.pnm", "output/test3.2_Basic");
  WaveNav::PathPlannerOutput path32b = findAnyPath(myNav, "Basic", topLeft, upperMiddle);
  printOutput(path32b);

  std::cout << "\ntest4\n";
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.1_OFWF");
  WaveNav::PathPlannerOutput path41o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path41o);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.1_Basic");
  WaveNav::PathPlannerOutput path41b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path41b);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.2_OFWF");
  WaveNav::PathPlannerOutput path42o = findAnyPath(myNav, "OFWF", midLeft, midRight);
  printOutput(path42o);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.2_Basic");
  WaveNav::PathPlannerOutput path42b = findAnyPath(myNav, "Basic", midLeft, midRight);
  printOutput(path42b);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.3_OFWF");
  WaveNav::PathPlannerOutput path43o = findAnyPath(myNav, "OFWF", midLeft, middle);
  printOutput(path43o);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.3_Basic");
  WaveNav::PathPlannerOutput path43b = findAnyPath(myNav, "Basic", midLeft, middle);
  printOutput(path43b);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.4_OFWF");
  WaveNav::PathPlannerOutput path44o = findAnyPath(myNav, "OFWF", middle, midLeft);
  printOutput(path44o);
  myNav = WaveNav("../bitmaps/2dmap-test4.pnm", "output/test4.4_Basic");
  WaveNav::PathPlannerOutput path44b = findAnyPath(myNav, "Basic", middle, midLeft);
  printOutput(path44b);

  std::cout << "\ntest5\n";
  myNav = WaveNav("../bitmaps/2dmap-test5.pnm", "output/test5.1_OFWF");
  WaveNav::PathPlannerOutput path51o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path51o);
  myNav = WaveNav("../bitmaps/2dmap-test5.pnm", "output/test5.1_Basic");
  WaveNav::PathPlannerOutput path51b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path51b);
  myNav = WaveNav("../bitmaps/2dmap-test5.pnm", "output/test5.2_OFWF");
  WaveNav::PathPlannerOutput path52o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
  printOutput(path52o);
  myNav = WaveNav("../bitmaps/2dmap-test5.pnm", "output/test5.2_Basic");
  WaveNav::PathPlannerOutput path52b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
  printOutput(path52b);
  myNav = WaveNav("../bitmaps/2dmap-test5.pnm", "output/test5.3_OFWF");
  WaveNav::PathPlannerOutput path53o = findAnyPath(myNav, "OFWF", bottomLeft, middle);
  printOutput(path53o);
  myNav = WaveNav("../bitmaps/2dmap-test5.pnm", "output/test5.3_Basic");
  WaveNav::PathPlannerOutput path53b = findAnyPath(myNav, "Basic", bottomLeft, middle);
  printOutput(path53b);

  std::cout << "\ntest6\n";
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.1_OFWF");
  WaveNav::PathPlannerOutput path61o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path61o);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.1_Basic");
  WaveNav::PathPlannerOutput path61b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path61b);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.2_OFWF");
  WaveNav::PathPlannerOutput path62o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
  printOutput(path62o);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.2_Basic");
  WaveNav::PathPlannerOutput path62b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
  printOutput(path62b);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.3_OFWF");
  WaveNav::PathPlannerOutput path63o = findAnyPath(myNav, "OFWF", bottomLeft, centerLeft);
  printOutput(path63o);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.3_Basic");
  WaveNav::PathPlannerOutput path63b = findAnyPath(myNav, "Basic", bottomLeft, centerLeft);
  printOutput(path63b);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.4_OFWF");
  WaveNav::PathPlannerOutput path64o = findAnyPath(myNav, "OFWF", topLeft, centerLeft);
  printOutput(path64o);
  myNav = WaveNav("../bitmaps/2dmap-test6.pnm", "output/test6.4_Basic");
  WaveNav::PathPlannerOutput path64b = findAnyPath(myNav, "Basic", topLeft, centerLeft);
  printOutput(path64b);

}




