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
  WaveNav();
  WaveNav(std::string filename, std::string outName);
  ~WaveNav();
  void changeOutPath(std::string newOutName);
  std::pair<std::vector<GridCell>, double> drawSmoothedPath();
  PathPlannerOutput planPath(GridCell &start, GridCell &goal, const std::string& waveType);
  void printCells(std::list<GridCell> cells);
  void markCell(const GridCell& cell, long value);
  void markCells(const std::list<GridCell>& cells, long value);
  void markCells(const std::vector<GridCell> &cells, long value);

private:
  std::string outPath;
  std::list<GridCell> wayCells;
  std::list<GridCell> smoothedPath;
  OccGrid gridMap;
  OccGrid debugGrid;
  std::string mapfilename;
  void resetNavigator();
  void calcWayCells(const GridCell &start, const GridCell &goal);
  GridCell findNextCell(const GridCell &curr);
  void smoothestPath();
  void smoothPath();
  long smoothPathHelper();
  long smoothPathHelper2();
  long smoothPathHelper3();
  long smoothPathHelperReverse();
  bool isInLine(const GridCell &c1, const GridCell &c2);
  std::vector<GridCell> drawLine(const GridCell &c0, const GridCell &c1);
  double euclideanDist(const GridCell &c0, const GridCell &c1);
};


#endif //WAVEFRONT_WAVENAV_H
