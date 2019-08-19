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

#include "pathplanning/Coordinates.h"
#include "pathplanning/OccGrid.h"
#include "pathplanning/DebugGrid.h"

/**
 * WaveNav class handles the path planning. A WaveNav object is created with an occupancy map file
 * and an output path stub.
 *
 * A WaveNav object has two OccGrids. One is used to plan paths, the other is used to display
 * visualizations.
 *
 * After planPath() and smoothPath() are called, the WaveNav object stores the paths in instance
 * variables (initialPath_ and smoothedPath_)
 *
 * Usage:
 *   1) Construct a WaveNave object with a occupancy map file.
 *   2) Pick a source and target (will take any of the 3 types of coordinates -- GridCell, GPS, or x,y)
 *   3) planPath(source, target, <wavefront type>, visualization_flag)
 *      wavefront type is either "Basic" or "OFWF"
 *   4) use getInitialPath() and getSmoothedPath() to return lists of GridCells that lie on the paths.
 *      You must convert the GridCell coordinates to whichever coordinate system you are using.
 */


namespace pathplanner {

class WaveNav {

public:
  struct ppOutput {
    std::string waveType_;
    int numCellsVisited_;
    double initialPathLength_;
    double smoothPathLength_;
    double cpuTime_;
    double searchTime_;
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
  GridCell findMinNeighbor(const GridCell &curr);
  void smoothPath();
  void smoothPathHelper();
  void addDistanceWaypoints(double distanceInMeters);
  double getSmoothedPathLength();
};

}
#endif //WAVEFRONT_WAVENAV_H
