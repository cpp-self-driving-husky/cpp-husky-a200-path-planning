
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
  void markStart(const GridCell &start);
  void markGoal(const GridCell &goal);
  void myLine(GridCell &start, GridCell &end, cv::Vec3b &color);

private:
  int gridWidth_, gridHeight_;
  cv::Mat mat_;
};

}
#endif //WAVEFRONT_DEBUGGRID_H
