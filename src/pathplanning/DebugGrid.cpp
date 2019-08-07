//
// Created by allen on 7/8/19.
//

#include "pathplanning/DebugGrid.h"

namespace pathplanner {

DebugGrid::DebugGrid() = default;
//  gridWidth_ = GRID_WIDTH;
//  gridHeight_ = GRID_HEIGHT;
//  colorGrid_.resize(GRID_HEIGHT);
//  Pixel fill = Pixel(255);
//  for (auto &row : colorGrid_) {
//    row.resize(GRID_WIDTH, fill);
//  }



DebugGrid::DebugGrid(const std::string &filename) {
  mat_ = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  gridHeight_ = mat_.rows;
  gridWidth_ = mat_.cols;
  originalMat_ = mat_.clone();
}

//DebugGrid::DebugGrid(const std::string &filename, double mapScale) {
//  mat_ = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//  gridHeight_ = mat_.rows;
//  gridWidth_ = mat_.cols;


//  std::string fileFormat;
//  int inputWidth, inputHeight, maxVal;
//  Pixel currGridPixel;
//  int gridRow, gridCol;
//
//  std::ifstream ifs;
//  ifs.open(filename.c_str(), std::ifstream::in);
//  ifs >> fileFormat;
//  ifs >> inputWidth >> inputHeight;
//  gridWidth_ = static_cast<int>(std::ceil(inputWidth / mapScale));
//  gridHeight_ = static_cast<int>(std::ceil(inputHeight / mapScale));
//  colorGrid_.resize(gridHeight_);
//  Pixel whitePixel = Pixel(255);
//  for (auto &row : colorGrid_) {
//    row.resize(gridWidth_, whitePixel);
//  }
//
//  if (fileFormat == "P1") {
//    char nextChar;
//    int nextInt;
//    for (int inRow = 0; inRow < inputHeight; ++inRow) {
//      for (int inCol = 0; inCol < inputWidth; ++inCol) {
//        ifs >> nextChar;
//        nextInt = nextChar - '0';
//        gridRow = static_cast<int>(inRow / mapScale);
//        gridCol = static_cast<int>(inCol / mapScale);
//        currGridPixel = colorGrid_[gridRow][gridCol];
//        if (currGridPixel.isWhite()) {
//          if (nextInt == 1) {
//            colorGrid_[gridRow][gridCol] = Pixel(0);
//          }
//        }
//      }
//    }
//  } else if (fileFormat == "P2") {
//    ifs >> maxVal;
//    int nextInt;
//    for (int inRow = 0; inRow < inputHeight; ++inRow) {
//      for (int inCol = 0; inCol < inputWidth; ++inCol) {
//        ifs >> nextInt;
//        gridRow = static_cast<int>(inRow / mapScale);
//        gridCol = static_cast<int>(inCol / mapScale);
//        currGridPixel = colorGrid_[gridRow][gridCol];
//        if (!currGridPixel.isBlack()) {
//          colorGrid_[gridRow][gridCol] = Pixel(nextInt);
//        }
//      }
//    }
//  } else if (fileFormat == "P5") {
//    ifs >> maxVal;
//    char nextChar;
//    for (int inRow = 0; inRow < inputHeight; ++inRow) {
//      for (int inCol = 0; inCol < inputWidth; ++inCol) {
//        ifs >> nextChar;
//        gridRow = static_cast<int>(inRow / mapScale);
//        gridCol = static_cast<int>(inCol / mapScale);
//        currGridPixel = colorGrid_[gridRow][gridCol];
//        if (!currGridPixel.isBlack()) {
//          colorGrid_[gridRow][gridCol] = Pixel(nextChar);
//        }
//      }
//    }
//  } else {
//    std::cout << "Can't read image file format." << std::endl;
//  }
//  ifs.close();
//}


void DebugGrid::outputGrid(const std::string &outputPath) {
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(3);
  cv::imwrite(outputPath, mat_, compression_params);
//  std::ofstream outFile(outputPath.c_str(), std::ofstream::out);
//  outFile << "P3" << std::endl;
//  outFile << gridWidth_ << " " << gridHeight_ << std::endl << 255 << std::endl;
//
//  for (int row = 0; row < gridHeight_; ++row) {
//    for (int col = 0; col < gridWidth_; ++col) {
//      outFile << colorGrid_[row][col].toString() << " ";
//    }
//  }
//  outFile.close();
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


//  int startRow = start.getRow();
//  int startCol = start.getCol();
//
//  std::vector<GridCell> startCells;
//  startCells.emplace_back(start);
//  startCells.emplace_back(GridCell(startCol - 1, startRow));
//  startCells.emplace_back(GridCell(startCol + 1, startRow));
//  startCells.emplace_back(GridCell(startCol, startRow - 1));
//  startCells.emplace_back(GridCell(startCol, startRow + 1));
//
//  startCells.emplace_back(GridCell(startCol - 1, startRow - 3));
//  startCells.emplace_back(GridCell(startCol, startRow - 3));
//  startCells.emplace_back(GridCell(startCol + 1, startRow - 3));
//
//  startCells.emplace_back(GridCell(startCol - 1, startRow + 3));
//  startCells.emplace_back(GridCell(startCol, startRow + 3));
//  startCells.emplace_back(GridCell(startCol + 1, startRow + 3));
//
//  startCells.emplace_back(GridCell(startCol - 3, startRow - 1));
//  startCells.emplace_back(GridCell(startCol - 3, startRow));
//  startCells.emplace_back(GridCell(startCol - 3, startRow + 1));
//
//  startCells.emplace_back(GridCell(startCol + 3, startRow - 1));
//  startCells.emplace_back(GridCell(startCol + 3, startRow));
//  startCells.emplace_back(GridCell(startCol + 3, startRow + 1));
//
//  startCells.emplace_back(GridCell(startCol - 2, startRow - 2));
//  startCells.emplace_back(GridCell(startCol - 2, startRow + 2));
//  startCells.emplace_back(GridCell(startCol + 2, startRow - 2));
//  startCells.emplace_back(GridCell(startCol + 2, startRow + 2));
//
//  cv::Vec3b color;
//  color[0] = 70;
//  color[1] = 70;
//  color[2] = 70;
//  markCells(startCells, color);

//  std::vector<GridCell> whiteCells;
//  whiteCells.emplace_back(GridCell(startCol - 1, startRow - 2));
//  whiteCells.emplace_back(GridCell(startCol, startRow - 2));
//  whiteCells.emplace_back(GridCell(startCol + 1, startRow - 2));
//
//  whiteCells.emplace_back(GridCell(startCol - 1, startRow + 2));
//  whiteCells.emplace_back(GridCell(startCol, startRow + 2));
//  whiteCells.emplace_back(GridCell(startCol + 1, startRow + 2));
//
//  whiteCells.emplace_back(GridCell(startCol - 2, startRow - 1));
//  whiteCells.emplace_back(GridCell(startCol - 2, startRow));
//  whiteCells.emplace_back(GridCell(startCol - 2, startRow + 1));
//
//  whiteCells.emplace_back(GridCell(startCol + 2, startRow - 1));
//  whiteCells.emplace_back(GridCell(startCol + 2, startRow));
//  whiteCells.emplace_back(GridCell(startCol + 2, startRow + 1));
//
//  whiteCells.emplace_back(GridCell(startCol - 1, startRow - 1));
//  whiteCells.emplace_back(GridCell(startCol - 1, startRow + 1));
//  whiteCells.emplace_back(GridCell(startCol + 1, startRow - 1));
//  whiteCells.emplace_back(GridCell(startCol + 1, startRow + 1));
//
//  color[0] = 255;
//  color[1] = 255;
//  color[2] = 255;
//  markCells(whiteCells, color);





//
//  int goalRow = goal.getRow();
//  int goalCol = goal.getCol();
//
//  std::vector<GridCell> goalCells;
//
//  for (int col = goalCol - 2; col <= goalCol + 2; ++col) {
//    goalCells.emplace_back(GridCell(col, goalRow - 2));
//    goalCells.emplace_back(GridCell(col, goalRow + 2));
//  }
//
//  for (int row = goalRow - 1; row <= goalRow + 1; ++row) {
//    goalCells.emplace_back(GridCell(goalCol - 2, row));
//    goalCells.emplace_back(GridCell(goalCol + 2, row));
//  }
//
//  std::vector<GridCell> whiteCells;
//  for (int row = goalRow - 1; row <= goalRow + 1; ++row) {
//    for (int col = goalCol - 1; col <= goalCol + 1; ++col) {
//      whiteCells.emplace_back(GridCell(col, row));
//    }
//  }
//
//  cv::Vec3b color;
//  color[0] = 255;
//  color[1] = 255;
//  color[2] = 255;
//  markCells(whiteCells, color);
//
//  goalCells.emplace_back(goal);
//  color[0] = 70;
//  color[1] = 70;
//  color[2] = 70;
//  markCells(goalCells, color);


void DebugGrid::myLine(cv::Mat &mat, GridCell &start, GridCell &end, cv::Vec3b &color) {
  int thickness = gridWidth_/200;
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
  startColor[1] = 195;
  startColor[2] = 255;

  cv::Vec3b goalColor;
  goalColor[0] = 0;
  goalColor[1] = 255;
  goalColor[2] = 0;

  cv::circle(tempMat, startCenter, gridWidth_ / 100, goalColor, gridWidth_ / 300, cv::LINE_AA, 0);
  cv::circle(tempMat, goalCenter, gridWidth_ / 100, goalColor, gridWidth_ / 100, cv::LINE_AA, 0);


  cv::Vec3b color;
  cv::Vec3b waypointColor;
  if (waveType == "OFWF") {
    color[0] = 0;
    color[1] = 195;
    color[2] = 255;
    waypointColor[0] = 140;
    waypointColor[1] = 20;
    waypointColor[2] = 60;
  } else {
    color[0] = 255;
    color[1] = 40;
    color[2] = 40;
    waypointColor[0] = 0;
    waypointColor[1] = 255;
    waypointColor[2] = 255;
  }


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
  }



  double opacity = 0.9;
  cv::addWeighted(tempMat, opacity, mat_, 1 - opacity, 0, mat_);

//  debugGrid_.markStart(smoothedPath_.front());
//  debugGrid_.markGoal(smoothedPath_.back());
}


void DebugGrid::markPaths(std::list<GridCell> &initialP, std::list<GridCell> &smoothP, const std::string &waveType) {
  cv::Mat wavyMat = mat_.clone();

  markSmoothedPath(smoothP, waveType);
  markInitialPath(initialP);
}

}
