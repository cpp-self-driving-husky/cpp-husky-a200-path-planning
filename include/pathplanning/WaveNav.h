/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WavefrontNavigator.h
 */


#ifndef WAVEFRONT_WAVENAV_H
#define WAVEFRONT_WAVENAV_H

#include <chrono>
#include <climits>
#include <cmath>
#include <exception>
#include <iostream>
#include <fstream>
#include <list>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>

#include "pathplanning/Coordinates.h"
#include "pathplanning/OccGrid.h"
#include "pathplanning/DebugGrid.h"

namespace pathplanner {

class WaveNav {

public:
  struct PathPlannerOutput {
    std::string waveType;
    int numCellsVisited;
    double initialPathLength;
    double smoothPathLength;
    double cpuTime;
  };
  WaveNav(std::string inputPath, std::string outputPathPrefix);
  ~WaveNav();
  double markSmoothedPath();
  PathPlannerOutput planPath(GridCell &start, GridCell &goal, const std::string &waveType);
  PathPlannerOutput planPath(Point &start, Point &goal, const std::string &waveType);
  PathPlannerOutput planPath(GPS &start, GPS &goal, const std::string &waveType);
  static void printCells(const std::list<GridCell> &cells);
  static void printCells(const std::vector<GridCell> &cells);

private:
  std::string inputPath_;
  std::string outputPathPrefix_;
  OccGrid gridMap_;
  DebugGrid debugGrid_;
  std::list<GridCell> initialPath_;
  std::list<GridCell> smoothedPath_;
  std::unordered_map<std::string, Point> destinations_;
  void findInitialPath(const GridCell &start, const GridCell &goal);
  GridCell findMinNeighbor(const GridCell &curr);
  void smoothePath();
  void smoothePathHelper();
  void loadDestinations(std::string &filename);
};

}
#endif //WAVEFRONT_WAVENAV_H
