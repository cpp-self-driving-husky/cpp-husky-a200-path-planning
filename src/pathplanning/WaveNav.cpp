/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WaveNav.cpp
 */

#include "pathplanning/WaveNav.h"

namespace pathplanner {

WaveNav::WaveNav(const std::string &inputPath, const std::string &outputPathPrefix) {
  inputPath_ = inputPath;
  outputPathPrefix_ = outputPathPrefix;
  gridMap_ = OccGrid(inputPath_, 1);
//  gridMap_.outputGrid(outputPathPrefix_ + "_1-scaled input map.pnm");
//  gridMap_ = gridMap_.growGrid(0.2);
//  debugGrid_ = DebugGrid(outputPathPrefix_ + "_1-scaled input map.pnm");
  debugGrid_ = DebugGrid(inputPath);
  initialPath_.clear();
  smoothedPath_.clear();
}


WaveNav::~WaveNav() = default;


WaveNav::ppOutput WaveNav::planPath(GridCell &start, GridCell &goal, const std::string &waveType, int debugLevel) {
  WaveNav::ppOutput toReturn;
  toReturn.waveType_ = waveType;
  gridMap_.normCell(start);
  gridMap_.normCell(goal);

//  OccGrid grownGrid = gridMap_;

  auto begin = std::chrono::high_resolution_clock::now();
  auto waveResult = (waveType == "Basic") ?
                    gridMap_.propWavesBasic(goal, start, 5, 7) :
                    gridMap_.propOFWF(goal, start, 5, 7);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - begin);
  toReturn.cpuTime_ = duration.count();

  GridCell finalCell = waveResult.first;
  toReturn.numCellsVisited_ = waveResult.second;
  toReturn.initialPathLength_ = static_cast<double>(gridMap_.get(finalCell)) * SCALE_MAP / 50;

//  if (debugLevel == 1) {
//    std::string waveColor = (waveType == "Basic") ? "R" : "B";
//    debugGrid_.markWaves(gridMap_, waveColor);
//    debugGrid_.outputGrid(outputPathPrefix_ + "_2-debug.png");
//  }

  if (finalCell.equals(start)) {
    findInitialPath(finalCell, goal);

//    gridMap_ = OccGrid(outputPathPrefix_ + "_1-scaled input map.pnm", 1.0);
//    gridMap_ = gridMap_.growGrid(0.15);

    smoothePath();
    toReturn.smoothPathLength_ = getSmoothedPathLength();

    auto end = std::chrono::high_resolution_clock::now();
    auto searchDuration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
    toReturn.searchTime_ = searchDuration.count();

  }
  if (debugLevel == 1) {
    std::string waveColor = (waveType == "Basic") ? "R" : "B";
    debugGrid_.markWaves(gridMap_, waveColor);
    debugGrid_.markPaths(initialPath_, smoothedPath_, waveType);
    debugGrid_.outputGrid(outputPathPrefix_ + "_2-debug.png");
  }
  return toReturn;
}


WaveNav::ppOutput WaveNav::planPath(Point &start, Point &goal, const std::string &waveType, int debugLevel) {
  GridCell startCell = pointToCell(start);
  GridCell goalCell = pointToCell(goal);
  return WaveNav::planPath(startCell, goalCell, waveType, debugLevel);
}


WaveNav::ppOutput WaveNav::planPath(GPS &start, GPS &goal, const std::string &waveType, int debugLevel) {
  GridCell startCell = gpsToCell(start);
  GridCell goalCell = gpsToCell(goal);
  return WaveNav::planPath(startCell, goalCell, waveType, debugLevel);
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


std::vector<GridCell> WaveNav::getInitialPath() {
  return std::vector<GridCell> {initialPath_.begin(), initialPath_.end()};
}


std::vector<GridCell> WaveNav::getSmoothedPath() {
  return std::vector<GridCell> {smoothedPath_.begin(), smoothedPath_.end()};
}


double WaveNav::getSmoothedPathLength() {
  double pathLength = 0.0;
  auto it0 = smoothedPath_.begin();
  auto it1 = it0;
  ++it1;
  while (it1 != smoothedPath_.end()) {
    pathLength += OccGrid::euclideanDistMeters(*it0, *it1);
    ++it0;
    ++it1;
  }
  return pathLength;
}

}
