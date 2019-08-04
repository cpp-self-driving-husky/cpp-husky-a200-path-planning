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

//#include <boost/filesystem.hpp>

#include "pathplanning/Coordinates.h"
#include "pathplanning/OccGrid.h"
#include "pathplanning/DebugGrid.h"

namespace pathplanner {

class WaveNav {

public:
  struct ppOutput {
    std::string waveType_;
    int numCellsVisited_;
    double initialPathLength_;
    double smoothPathLength_;
    double cpuTime_;
  };
  WaveNav() = default;
  WaveNav(const std::string &inputPath, const std::string &outputPathPrefix);
  ~WaveNav();
  WaveNav::ppOutput planPath(GridCell &start, GridCell &goal, const std::string &waveType, int debugLevel);
  ppOutput planPath(Point &start, Point &goal, const std::string &waveType, int debugLevel);
  ppOutput planPath(GPS &start, GPS &goal, const std::string &waveType, int debugLevel);
  std::vector<GridCell> getInitialPath();
  std::vector<GridCell> getSmoothedPath();

private:
  std::string inputPath_;
  std::string outputPathPrefix_;
  OccGrid gridMap_;
  DebugGrid debugGrid_;
  std::list<GridCell> initialPath_;
  std::list<GridCell> smoothedPath_;

  void findInitialPath(const GridCell &start, const GridCell &goal);
  void markInitialPath();
  GridCell findMinNeighbor(const GridCell &curr);
  void smoothePath();
  void smoothePathHelper();
  void markSmoothedPath(const std::string &waveType);
  double getSmoothedPathLength();
};

}
#endif //WAVEFRONT_WAVENAV_H
