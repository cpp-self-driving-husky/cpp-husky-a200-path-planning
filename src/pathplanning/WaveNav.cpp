/**
 * project: CATE
 *    team: Behavioral (Path Planning)
 *  author: Allen Kim
 *    file: WaveNav.cpp
 */

#include "../../include/pathplanning/WaveNav.h"

namespace pathplanner {

WaveNav::WaveNav(const std::string &inputPath, const std::string &outputPathPrefix) {
  inputPath_ = inputPath;
  outputPathPrefix_ = outputPathPrefix;
  gridMap_ = OccGrid(inputPath_, 1);
  if (inputPath_.substr(11, 5) == "grown") {
    visualizationGrid_ = PathVisualization("../bitmaps/" + inputPath_.substr(17));
  } else {
    gridMap_ = gridMap_.growGrid(0.5);
    visualizationGrid_ = PathVisualization(inputPath_);
  }
  initialPath_.clear();
  smoothedPath_.clear();
  maxDistanceBetweenWaypoints_ = 5.0;
}


/**
 * Finds the optimal initial path from start cell to goal cell. Applies smoothing to initial path.
 * Stores these two paths in instance variables initialPath_ and smoothedPath_. Also adds waypoints
 * to the smoothedPath_ so that no 2 waypoints are farther apart than maxDistanceBetweenWaypoints_.
 * Use getInitialPath() or getSmoothedPath() to get a vector of waypoints.
 *
 * If showVisualization is true, displays a map with waves and paths and writes this map to an image file.
 *
 * @param start
 * @param goal
 * @param waveType
 * @param showVisualization
 * @return WaveNav::ppOutput is a struct that bundles various measurements together. This is used to
 *         gather stats for reporting and testing.
 */
WaveNav::ppOutput WaveNav::planPath(GridCell &start, GridCell &goal, const std::string &waveType, bool showVisualization) {
  WaveNav::ppOutput toReturn;
  toReturn.waveType_ = waveType;
  gridMap_.normCell(start);
  gridMap_.normCell(goal);

  auto begin = std::chrono::high_resolution_clock::now();
  auto waveResult = (waveType == "Basic") ?
                    gridMap_.propWavesBasic(goal, start, 5, 7) :
                    gridMap_.propWavesOFWF(goal, start, 5, 7);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - begin);
  toReturn.cpuTime_ = duration.count();

  GridCell finalCell = waveResult.first;
  toReturn.numCellsVisited_ = waveResult.second;
  toReturn.initialPathLength_ = static_cast<double>(gridMap_.get(finalCell) - 2) * SCALE_MAP / 50;

  if (finalCell.equals(start)) {
    findInitialPath(finalCell, goal);
    smoothPath();
    toReturn.smoothPathLength_ = getSmoothedPathLength();

    addDistanceWaypoints();

    auto end = std::chrono::high_resolution_clock::now();
    auto searchDuration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
    toReturn.searchTime_ = searchDuration.count();

  }
  if (showVisualization) {
    std::string waveColor = (waveType == "Basic") ? "R" : "B";
    visualizationGrid_.markWaves(gridMap_, waveColor);
    visualizationGrid_.markPaths(initialPath_, smoothedPath_, waveType);
    visualizationGrid_.outputVis(outputPathPrefix_ + "_vis.png");
    visualizationGrid_.displayVis();
  }
  return toReturn;
}


/**
 * Overloaded planPath() for (x, y) coordinates
 *
 * @param start
 * @param goal
 * @param waveType
 * @param showVisualization
 * @return
 */
WaveNav::ppOutput WaveNav::planPath(Point &start, Point &goal, const std::string &waveType, bool showVisualization) {
  GridCell startCell = pointToCell(start);
  GridCell goalCell = pointToCell(goal);
  return WaveNav::planPath(startCell, goalCell, waveType, showVisualization);
}


/**
 * Overloaded planPath() for GPS coordinates
 *
 * @param start
 * @param goal
 * @param waveType
 * @param showVisualization
 * @return
 */
WaveNav::ppOutput WaveNav::planPath(GPS &start, GPS &goal, const std::string &waveType, bool showVisualization) {
  GridCell startCell = gpsToCell(start);
  GridCell goalCell = gpsToCell(goal);
  return WaveNav::planPath(startCell, goalCell, waveType, showVisualization);
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
 * After wave propagation, repeatedly perform this function from the source node
 * to find the entire initial path.
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
 * Path simplification based on "string tightening" method.
 * From a jagged path where all turns are in multiples of 45 degrees, this removes
 * waypoints
 *
 * Solves the following: given a sequence of cells (c1, ... , cn) representing a path between
 * start and goal cells, the solution will satisfy:
 *    (1) has same start and goal cells
 *    (2) there is a clear line of sight between consecutive solution cells
 *    (3) total path length and number of nodes are minimized
 */
void WaveNav::smoothPath() {
  smoothedPath_.assign(initialPath_.begin(), initialPath_.end());
  int pathSizeBeforeSmooth, pathSizeAfterSmooth, smoothModifications;
  do {
    pathSizeBeforeSmooth = smoothedPath_.size();
    smoothPathHelper();
    pathSizeAfterSmooth = smoothedPath_.size();
    smoothModifications = pathSizeBeforeSmooth - pathSizeAfterSmooth;
  } while (smoothModifications > 0);
}


void WaveNav::smoothPathHelper() {
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


/**
 * Adds intermediate waypoints into the smoothedPath_ so that the distance
 * between consecutive waypoints is always less than or equal to distanceInMeters.
 *
 * @param distanceInMeters
 */
void WaveNav::addDistanceWaypoints() {
  auto it0 = smoothedPath_.begin();
  auto it1 = smoothedPath_.begin();
  ++it1;
  auto it_end = smoothedPath_.end();

  while (it1 != it_end) {
    double lineLength = OccGrid::euclideanDistMeters(*it0, *it1);
    if (lineLength > maxDistanceBetweenWaypoints_) {
      std::vector<GridCell> line = OccGrid::drawLine(*it0, *it1);
      int pointsToAdd = std::ceil(lineLength / maxDistanceBetweenWaypoints_);
      auto pointSelector = line.size() / pointsToAdd;
      for (int i = 1; i * pointSelector < line.size(); ++i) {
        smoothedPath_.insert(it1, line.at(pointSelector * i));
      }
    }
    it0 = it1;
    ++it1;
  }
}

std::vector<GridCell> WaveNav::getInitialPath() {
  return std::vector<GridCell> {initialPath_.begin(), initialPath_.end()};
}


std::vector<GridCell> WaveNav::getSmoothedPath() {
  return std::vector<GridCell> {smoothedPath_.begin(), smoothedPath_.end()};
}


/**
 * Finds the length of the smoothed path as the sum of euclidean distances
 * between successive waypoints.
 *
 * @return
 */
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
