/**
 * project: CATE
 *    team: Behavioral (Path Planning)
 *  author: Allen Kim
 *    file: PathVisualization.h
 */

#ifndef WAVEFRONT_PATHVISUALIZATION_H
#define WAVEFRONT_PATHVISUALIZATION_H

#include <opencv2/opencv.hpp>

#include "OccGrid.h"

namespace pathplanner {

class PathVisualization {
public:
  PathVisualization();
  explicit PathVisualization(const std::string &filename);
  virtual ~PathVisualization() = default;
  void outputVis(const std::string &outputPath);
  void displayVis();
  void markWaves(const OccGrid &waveGrid, const std::string &waveColor);
  void markCell(const GridCell &cell, const cv::Vec3b &color);
  void markCells(const std::vector<GridCell> &cells, const cv::Vec3b &color);
  void markCells(const std::list<GridCell> &cells, const cv::Vec3b &color);
  void markInitialPath(std::list<GridCell> path);
  void markSmoothedPath(std::list<GridCell> path, const std::string &waveType);
  void markPaths(std::list<GridCell> &initialP, std::list<GridCell> &smoothP, const std::string &waveType);

private:
  int gridWidth_, gridHeight_;
  cv::Mat mat_;
  cv::Mat originalMat_;

  void myLine(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color);
  void myLineThin(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color);
};

}
#endif //WAVEFRONT_PATHVISUALIZATION_H
