/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WaveNav.cpp
 */

#include "pathplanning/WaveNav.h"

namespace pathplanner {

WaveNav::WaveNav(std::string inputPath, std::string outputPathPrefix) {
  inputPath_ = std::move(inputPath);
  outputPathPrefix_ = std::move(outputPathPrefix);
  gridMap_ = OccGrid(inputPath_, SCALE_MAP);
  gridMap_.outputGrid(outputPathPrefix_ + "_1-scaled input map.pnm");
  gridMap_ = gridMap_.growGrid(0.50);
  debugGrid_ = DebugGrid(outputPathPrefix_ + "_1-scaled input map.pnm", 1.0);
  initialPath_.clear();
  smoothedPath_.clear();
//  loadDestinations("../dest_coordinates.txt");
//  loadTestXY("../test_xy.txt");
}


WaveNav::~WaveNav() = default;


double WaveNav::markSmoothedPath() {
  double pathLength = 0.0;

  auto it0 = smoothedPath_.begin();
  auto it1 = it0;
  it1++;
  while (it1 != smoothedPath_.end()) {
    debugGrid_.markCells(OccGrid::drawLine(*it0, *it1), Pixel(255, 255, 0));
    pathLength += OccGrid::euclideanDist(*it0, *it1);
    it0++;
    it1++;
  }
  return pathLength;
}


WaveNav::PathPlannerOutput WaveNav::planPath(GridCell &start, GridCell &goal, const std::string &waveType) {
  WaveNav::PathPlannerOutput toReturn {};
  toReturn.waveType = waveType;
  gridMap_.normCell(start);
  gridMap_.normCell(goal);

  auto begin = std::chrono::high_resolution_clock::now();
  auto waveResult = (waveType == "Basic") ?
                    gridMap_.propWavesBasic(goal, start, 500, 707) :
                    gridMap_.propOFWF(goal, start, 500, 707);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - begin);
  toReturn.cpuTime = duration.count();

  GridCell finalCell = waveResult.first;
  toReturn.numCellsVisited = waveResult.second;
  toReturn.initialPathLength = static_cast<double>(gridMap_.get(finalCell)) / 500;

  if (waveType == "Basic") {
    debugGrid_.markWaves(gridMap_, "R");
  } else {
    debugGrid_.markWaves(gridMap_, "B");
  }

  if (finalCell.equals(start)) {
    findInitialPath(finalCell, goal);
    debugGrid_.markCells(initialPath_, Pixel(120));

    gridMap_ = OccGrid(outputPathPrefix_ + "_1-scaled input map.pnm", 1.0);
    gridMap_ = gridMap_.growGrid(0.2);

    smoothePath();

    toReturn.smoothPathLength = markSmoothedPath();

    std::vector<GridCell> neighborhood;
    neighborhood.reserve(9);
    for (const auto &cell : smoothedPath_) {
      gridMap_.getNeighborhood(cell, 1, neighborhood);
      debugGrid_.markCells(neighborhood, Pixel(0, 0, 255));
      neighborhood.clear();
    }

    debugGrid_.markStart(start);
    debugGrid_.markGoal(goal);

    debugGrid_.outputGrid(outputPathPrefix_ + "_2-debug.pnm");
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


void WaveNav::printCells(const std::list<GridCell> &cells) {
  for (const auto &cell : cells) {
    std::cout << "        " << cell.toString() << std::endl;
  }
}


void WaveNav::printCells(const std::vector<GridCell> &cells) {
  for (const auto &cell : cells) {
    std::cout << "        " << cell.toString() << std::endl;
  }
}


void WaveNav::loadDestinations(std::string &filename) {
  std::ifstream ifs;
  std::string name;
  double latitude, longitude;
  Point destination;

  ifs.open(filename.c_str(), std::ifstream::in);
  if (ifs) {
    while (ifs >> name) {
      ifs >> latitude >> longitude;
      destination = Point(longitudeToX(longitude), latitudeToY(latitude));
      destinations_.insert({name, destination});
    }
  }
  ifs.close();
}


/**
 * Finds initial path between start and goal cells after propagating waves.
 * Path is stored in initialPath instance variable.
 *
 * @param start
 * @param goal
 */
void WaveNav::findInitialPath(const GridCell &start, const GridCell &goal) {
  for (GridCell curr = start; curr != goal; curr = findMinNeighbor(curr)) {
    initialPath_.emplace_back(curr);
  }
  initialPath_.emplace_back(goal);
}


/**
 * Finds the neighboring cell with the lowest weight.
 * This shows the path as determined by the wavefront
 * propagation.
 *
 * @param curr - current GridCell
 * @return neighboring GridCell with lowest weight
 */
GridCell WaveNav::findMinNeighbor(const GridCell &curr) {
  int weight = gridMap_.get(curr);

  // if weight < 3, curr is the goal cell or is part of an obstacle
  if (weight < 3) {
    return curr;
  }

  // get cells in curr's neighborhood
  std::vector<GridCell> neighborhood;
  neighborhood.reserve(9);
  gridMap_.getNeighborhood(curr, 1, neighborhood);

  // find neighbor with lowest weight
  GridCell minNeighbor;
  int minWeight = INT_MAX;
  for (const auto &neighbor : neighborhood) {
    weight = gridMap_.get(neighbor);
    if ((weight < minWeight) && (weight > 1)) {
      minWeight = weight;
      minNeighbor = neighbor;
    }
  }
  if (minNeighbor == curr) {
    std::cout << "      $ ERROR: no neighboring cell has a lower value than current cell." << std::endl;
  }
  return minNeighbor;
}


/**
 * Path simplification which solves the following: given a sequence of cells (c1, ... , cn) representing a path between
 * start and goal cells, the solution will satisfy:
 *    (1) has same start and goal cells
 *    (2) there is a clear line of sight between consecutive solution cells
 *    (3) total path length and number of nodes are minimized
 */
void WaveNav::smoothePath() {
  smoothedPath_.assign(initialPath_.begin(), initialPath_.end());
  int pathSizeBeforeSmooth, pathSizeAfterSmooth, smoothModifications;
  do {
    pathSizeBeforeSmooth = smoothedPath_.size();
    smoothePathHelper();
    pathSizeAfterSmooth = smoothedPath_.size();
    smoothModifications = pathSizeBeforeSmooth - pathSizeAfterSmooth;
  } while (smoothModifications > 0);
}


void WaveNav::smoothePathHelper() {
  if (smoothedPath_.size() < 3) {
    return;
  }

  auto it0 = smoothedPath_.begin();
  auto it1 = it0;
  auto it2 = it0;
  auto it_end = smoothedPath_.end();
  ++it1;
  ++it2;
  ++it2;

//  while ((it0 != it_end) && (it1 != it_end) && (it2 != it_end) && (smoothedPath_.size() > 2)) {
  while ((it2 != it_end) && (smoothedPath_.size() > 2)) {
    if (gridMap_.isInLine(*it0, *it2)) {
      it1 = smoothedPath_.erase(it1);
      ++it2;
    } else {
      it0 = it1;
      ++it1;
      ++it2;
    }
  }
}

}
