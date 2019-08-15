//
// Created by allen on 7/8/19.
//

#include "pathplanning/DebugGrid.h"

namespace pathplanner {

DebugGrid::DebugGrid() = default;


DebugGrid::DebugGrid(const std::string &filename) {
  mat_ = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  gridHeight_ = mat_.rows;
  gridWidth_ = mat_.cols;
  originalMat_ = mat_.clone();
}


void DebugGrid::outputGrid(const std::string &outputPath) {
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(3);
  cv::imwrite(outputPath, mat_, compression_params);
  cv::imshow("Path Visualization", mat_);
  cv::waitKey(0);
}


void DebugGrid::markWaves(const OccGrid &waveGrid, const std::string &waveColor) {
  int maxDistance = waveGrid.findMaxDistance();
  int tintColor;

  Pixel white = Pixel(255);
  Pixel black = Pixel(0);

  for (int row = 0; row < gridHeight_; ++row) {
    for (int col = 0; col < gridWidth_; ++col) {
      if (waveGrid.get(col, row) > 1) {
        tintColor = static_cast<int>(((std::trunc(10.0 * (maxDistance - waveGrid.get(col, row)) / maxDistance)) / 10.0) * 255);
        if (waveColor == "R") {
          mat_.at<cv::Vec3b>(row, col)[0] = tintColor;
          mat_.at<cv::Vec3b>(row, col)[1] = tintColor;
          mat_.at<cv::Vec3b>(row, col)[2] = 255;
        } else if (waveColor == "G") {
          mat_.at<cv::Vec3b>(row, col)[0] = tintColor;
          mat_.at<cv::Vec3b>(row, col)[1] = 255;
          mat_.at<cv::Vec3b>(row, col)[2] = tintColor;
        } else if (waveColor == "B") {
          mat_.at<cv::Vec3b>(row, col)[0] = 255;
          mat_.at<cv::Vec3b>(row, col)[1] = tintColor;
          mat_.at<cv::Vec3b>(row, col)[2] = tintColor;
        } else if (waveColor == "Gray") {
        }
      }
    }
  }
}


void DebugGrid::markCell(const GridCell &cell, cv::Vec3b color) {
  int row {cell.getRow()};
  int col {cell.getCol()};
  if ((row >= 0) && (row < gridHeight_) && (col >= 0) && (col < gridWidth_)) {
    mat_.at<cv::Vec3b>(row, col) = color;
  }
}


void DebugGrid::markCells(const std::vector<GridCell> &cells, cv::Vec3b color) {
  for (const auto &cell : cells) {
    markCell(cell, color);
  }
}


void DebugGrid::markCells(const std::list<GridCell> &cells, cv::Vec3b color) {
  for (const auto &cell : cells) {
    markCell(cell, color);
  }
}


void DebugGrid::myLine(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color) {
  int thickness = gridWidth_/250;
  int lineType = cv::LINE_AA;
  cv::Point startPoint = cv::Point(start.getCol(), start.getRow());
  cv::Point endPoint = cv::Point(end.getCol(), end.getRow());
  cv::line(mat,
           startPoint,
           endPoint,
           color,
           thickness,
           lineType);
}


void DebugGrid::myLineThin(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color) {
  int thickness = gridWidth_/500;
  int lineType = cv::LINE_AA;
  cv::Point startPoint = cv::Point(start.getCol(), start.getRow());
  cv::Point endPoint = cv::Point(end.getCol(), end.getRow());
  cv::line(mat,
           startPoint,
           endPoint,
           color,
           thickness,
           lineType);
}

void DebugGrid::markInitialPath(std::list<GridCell> path) {
  cv::Vec3b color;
  color[0] = 255;
  color[1] = 255;
  color[2] = 255;

  cv::Mat tempMat = mat_.clone();

  auto it0 = path.begin();
  auto it1 = it0;
  ++it1;

  while (it1 != path.end()) {
    myLineThin(tempMat, *it0, *it1, color);
    ++it0;
    ++it1;
  }

  double opacity = 0.9;
  cv::addWeighted(tempMat, opacity, mat_, 1 - opacity, 0, mat_);
}


void DebugGrid::markSmoothedPath(std::list<GridCell> path, const std::string &waveType) {

  cv::Mat tempMat = mat_.clone();

  // mark start and goal
  cv::Point startCenter {path.front().getCol(), path.front().getRow()};
  cv::Point goalCenter {path.back().getCol(), path.back().getRow()};

  cv::Vec3b startColor;
  startColor[0] = 0;
  startColor[1] = 255;
  startColor[2] = 255;

  cv::Vec3b goalColor;
  goalColor[0] = 0;
  goalColor[1] = 255;
  goalColor[2] = 0;

  cv::circle(tempMat, startCenter, gridWidth_ / 100, startColor, gridWidth_ / 300, cv::LINE_AA, 0);
  cv::circle(tempMat, goalCenter, gridWidth_ / 100, goalColor, gridWidth_ / 100, cv::LINE_AA, 0);


  cv::Vec3b color;
  cv::Vec3b waypointColor;
  if (waveType == "OFWF") {
    color[0] = 0;
    color[1] = 0;
    color[2] = 255;
    waypointColor[0] = 0;
    waypointColor[1] = 0;
    waypointColor[2] = 0;
  } else {
    color[0] = 255;
    color[1] = 40;
    color[2] = 40;
    waypointColor[0] = 0;
    waypointColor[1] = 255;
    waypointColor[2] = 255;
  }

  cv::Vec3b white;
  white[0] = 255;
  white[1] = 255;
  white[2] = 255;


  auto it0 = path.begin();
  auto it1 = it0;
  ++it1;

  while (it1 != path.end()) {
    myLine(tempMat, *it0, *it1, color);
    ++it0;
    ++it1;
  }

  for (const auto &cell : path) {
    cv::Point curr {cell.getCol(), cell.getRow()};
    cv::circle(tempMat, curr, gridWidth_/200, waypointColor, -1, cv::LINE_AA, 0);
    cv::circle(tempMat, curr, gridWidth_/200, white, gridWidth_/700, cv::LINE_AA, 0);
  }

  double opacity = 0.9;
  cv::addWeighted(tempMat, opacity, mat_, 1 - opacity, 0, mat_);
}


void DebugGrid::markPaths(std::list<GridCell> &initialP, std::list<GridCell> &smoothP, const std::string &waveType) {
  cv::Mat wavyMat = mat_.clone();

  markSmoothedPath(smoothP, waveType);
  markInitialPath(initialP);
}

}
