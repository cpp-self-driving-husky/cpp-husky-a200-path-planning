/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: WavefrontNavigator.h
 */


#ifndef WAVEFRONT_WAVENAV_H
#define WAVEFRONT_WAVENAV_H

#include <chrono>
#include <string>
#include <unordered_map>
#include <list>
#include "../../include/pathplanning/OccGrid.h"

class WaveNav {

public:
  struct PathPlannerOutput {
    std::string waveType;
    long numCellsVisited;
    double initialPathLength;
    double smoothPathLength;
    double cpuTime;
  };
  WaveNav(std::string filename, std::string outName);
  ~WaveNav();
  void changeOutPath(std::string &newOutPath);
  std::pair<std::vector<GridCell>, double> drawSmoothedPath();
  PathPlannerOutput planPath(GridCell &start, GridCell &goal, const std::string& waveType);
  PathPlannerOutput planPath(Point &start, Point &goal, const std::string& waveType);
  PathPlannerOutput planPath(GPS &start, GPS &goal, const std::string& waveType);
  static void printCells(const std::list<GridCell>& cells);
  static void printCells(const std::vector<GridCell>& cells);

private:
  std::string mapfilename;
  std::string outPath;
  OccGrid gridMap;
  OccGrid debugGrid;
  std::list<GridCell> wayCells;
  std::list<GridCell> smoothedPath;
  std::unordered_map<std::string, Point> destinations;
  void initializeNavigator();
  void calcWayCells(const GridCell &start, const GridCell &goal);
  GridCell findNextCell(const GridCell &curr);
  void smoothPath();
  void smoothPathHelper();
  void loadDestinations(std::string& filename);
};


#endif //WAVEFRONT_WAVENAV_H
