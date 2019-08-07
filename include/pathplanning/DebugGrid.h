
//
// Created by allen on 7/8/19.
//

#ifndef WAVEFRONT_DEBUGGRID_H
#define WAVEFRONT_DEBUGGRID_H

#include <opencv2/opencv.hpp>

#include "OccGrid.h"
#include "Pixel.h"

namespace pathplanner {

class DebugGrid {
public:
  DebugGrid();
  explicit DebugGrid(const std::string &filename);
  virtual ~DebugGrid() = default;
  void outputGrid(const std::string &outputPath);
  void markWaves(const OccGrid &waveGrid, const std::string &waveColor);
  void markCell(const GridCell &cell, cv::Vec3b color);
  void markCells(const std::vector<GridCell> &cells, cv::Vec3b color);
  void markCells(const std::list<GridCell> &cells, cv::Vec3b color);
//  void markStart(const GridCell &start);
//  void markGoal(const GridCell &goal);
  void myLine(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color);
  void myLineThin(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color);
  void markInitialPath(std::list<GridCell> path);
  void markSmoothedPath(std::list<GridCell> path, const std::string &waveType);
  void markPaths(std::list<GridCell> &initialP, std::list<GridCell> &smoothP, const std::string &waveType);

private:
  int gridWidth_, gridHeight_;
  cv::Mat mat_;
  cv::Mat originalMat_;
};

}
#endif //WAVEFRONT_DEBUGGRID_H
