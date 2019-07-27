//
// Created by allen on 7/8/19.
//

#include "pathplanning/DebugGrid.h"

namespace pathplanner {

DebugGrid::DebugGrid() {
  gridWidth_ = GRID_WIDTH;
  gridHeight_ = GRID_HEIGHT;
  colorGrid_.resize(GRID_HEIGHT);
  Pixel fill = Pixel(255);
  for (auto &row : colorGrid_) {
    row.resize(GRID_WIDTH, fill);
  }
}


DebugGrid::DebugGrid(const std::string &filename, double mapScale) {
  std::string fileFormat;
  int inputWidth, inputHeight, maxVal;
  Pixel currGridPixel;
  int gridRow, gridCol;

  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ifstream::in);
  ifs >> fileFormat;
  ifs >> inputWidth >> inputHeight;
  gridWidth_ = static_cast<int>(std::ceil(inputWidth / mapScale));
  gridHeight_ = static_cast<int>(std::ceil(inputHeight / mapScale));
  colorGrid_.resize(gridHeight_);
  Pixel whitePixel = Pixel(255);
  for (auto &row : colorGrid_) {
    row.resize(gridWidth_, whitePixel);
  }

  if (fileFormat == "P1") {
    char nextChar;
    int nextInt;
    for (int inRow = 0; inRow < inputHeight; ++inRow) {
      for (int inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextChar;
        nextInt = nextChar - '0';
        gridRow = static_cast<int>(inRow / mapScale);
        gridCol = static_cast<int>(inCol / mapScale);
        currGridPixel = colorGrid_[gridRow][gridCol];
        if (currGridPixel.isWhite()) {
          if (nextInt == 1) {
            colorGrid_[gridRow][gridCol] = Pixel(0);
          }
        }
      }
    }
  } else if (fileFormat == "P2") {
    ifs >> maxVal;
    int nextInt;
    for (int inRow = 0; inRow < inputHeight; ++inRow) {
      for (int inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextInt;
        gridRow = static_cast<int>(inRow / mapScale);
        gridCol = static_cast<int>(inCol / mapScale);
        currGridPixel = colorGrid_[gridRow][gridCol];
        if (!currGridPixel.isBlack()) {
          colorGrid_[gridRow][gridCol] = Pixel(nextInt);
        }
      }
    }
  } else if (fileFormat == "P5") {
    ifs >> maxVal;
    char nextChar;
    for (int inRow = 0; inRow < inputHeight; ++inRow) {
      for (int inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextChar;
        gridRow = static_cast<int>(inRow / mapScale);
        gridCol = static_cast<int>(inCol / mapScale);
        currGridPixel = colorGrid_[gridRow][gridCol];
        if (!currGridPixel.isBlack()) {
          colorGrid_[gridRow][gridCol] = Pixel(nextChar);
        }
      }
    }
  } else {
    std::cout << "Can't read image file format." << std::endl;
  }
  ifs.close();
}


void DebugGrid::outputGrid(const std::string &outputPath) {
  std::ofstream outFile(outputPath.c_str(), std::ofstream::out);
  outFile << "P3" << std::endl;
  outFile << gridWidth_ << " " << gridHeight_ << std::endl << 255 << std::endl;

  for (int row = 0; row < gridHeight_; ++row) {
    for (int col = 0; col < gridWidth_; ++col) {
      outFile << colorGrid_[row][col].toString() << " ";
    }
  }
  outFile.close();
}


void DebugGrid::markWaves(const OccGrid &waveGrid, const std::string &waveColor) {
  int maxDistance = waveGrid.findMaxDistance();
  int tintColor;

  Pixel white = Pixel(255);
  Pixel black = Pixel(0);

  for (int row = 0; row < gridHeight_; ++row) {
    for (int col = 0; col < gridWidth_; ++col) {
      if (waveGrid.get(col, row) > 1) {
        tintColor =
            static_cast<int>(((std::trunc(10.0 * (maxDistance - waveGrid.get(col, row)) / maxDistance)) / 10.0) * 255);
        if (waveColor == "R") {
          colorGrid_[row][col] = Pixel(255, tintColor, tintColor);
        } else if (waveColor == "G") {
          colorGrid_[row][col] = Pixel(tintColor, 255, tintColor);
        } else if (waveColor == "B") {
          colorGrid_[row][col] = Pixel(tintColor, tintColor, 255);
        } else if (waveColor == "Gray") {
          colorGrid_[row][col] = Pixel(tintColor);
        }
      }
    }
  }
}


void DebugGrid::markCell(const GridCell &cell, const Pixel &color) {
  int row {cell.getRow()};
  int col {cell.getCol()};
  if ((row >= 0) && (row < gridHeight_) && (col >= 0) && (col < gridWidth_)) {
    colorGrid_[row][col] = color;
  }
}


void DebugGrid::markCells(const std::vector<GridCell> &cells, const Pixel &color) {
  for (const auto &cell : cells) {
    markCell(cell, color);
  }
}


void DebugGrid::markCells(const std::list<GridCell> &cells, const Pixel &color) {
  for (const auto &cell : cells) {
    markCell(cell, color);
  }
}


void DebugGrid::markStart(const GridCell &start) {
  int startRow = start.getRow();
  int startCol = start.getCol();

  std::vector<GridCell> startCells;
  startCells.emplace_back(start);
  startCells.emplace_back(GridCell(startCol - 1, startRow));
  startCells.emplace_back(GridCell(startCol + 1, startRow));
  startCells.emplace_back(GridCell(startCol, startRow - 1));
  startCells.emplace_back(GridCell(startCol, startRow + 1));

  startCells.emplace_back(GridCell(startCol - 1, startRow - 3));
  startCells.emplace_back(GridCell(startCol, startRow - 3));
  startCells.emplace_back(GridCell(startCol + 1, startRow - 3));

  startCells.emplace_back(GridCell(startCol - 1, startRow + 3));
  startCells.emplace_back(GridCell(startCol, startRow + 3));
  startCells.emplace_back(GridCell(startCol + 1, startRow + 3));

  startCells.emplace_back(GridCell(startCol - 3, startRow - 1));
  startCells.emplace_back(GridCell(startCol - 3, startRow));
  startCells.emplace_back(GridCell(startCol - 3, startRow + 1));

  startCells.emplace_back(GridCell(startCol + 3, startRow - 1));
  startCells.emplace_back(GridCell(startCol + 3, startRow));
  startCells.emplace_back(GridCell(startCol + 3, startRow + 1));

  startCells.emplace_back(GridCell(startCol - 2, startRow - 2));
  startCells.emplace_back(GridCell(startCol - 2, startRow + 2));
  startCells.emplace_back(GridCell(startCol + 2, startRow - 2));
  startCells.emplace_back(GridCell(startCol + 2, startRow + 2));

  markCells(startCells, Pixel(70));

  std::vector<GridCell> whiteCells;
  whiteCells.emplace_back(GridCell(startCol - 1, startRow - 2));
  whiteCells.emplace_back(GridCell(startCol, startRow - 2));
  whiteCells.emplace_back(GridCell(startCol + 1, startRow - 2));

  whiteCells.emplace_back(GridCell(startCol - 1, startRow + 2));
  whiteCells.emplace_back(GridCell(startCol, startRow + 2));
  whiteCells.emplace_back(GridCell(startCol + 1, startRow + 2));

  whiteCells.emplace_back(GridCell(startCol - 2, startRow - 1));
  whiteCells.emplace_back(GridCell(startCol - 2, startRow));
  whiteCells.emplace_back(GridCell(startCol - 2, startRow + 1));

  whiteCells.emplace_back(GridCell(startCol + 2, startRow - 1));
  whiteCells.emplace_back(GridCell(startCol + 2, startRow));
  whiteCells.emplace_back(GridCell(startCol + 2, startRow + 1));

  whiteCells.emplace_back(GridCell(startCol - 1, startRow - 1));
  whiteCells.emplace_back(GridCell(startCol - 1, startRow + 1));
  whiteCells.emplace_back(GridCell(startCol + 1, startRow - 1));
  whiteCells.emplace_back(GridCell(startCol + 1, startRow + 1));

  markCells(whiteCells, Pixel(255));
}


void DebugGrid::markGoal(const GridCell &goal) {
  int goalRow = goal.getRow();
  int goalCol = goal.getCol();

  std::vector<GridCell> goalCells;

  for (int col = goalCol - 2; col <= goalCol + 2; ++col) {
    goalCells.emplace_back(GridCell(col, goalRow - 2));
    goalCells.emplace_back(GridCell(col, goalRow + 2));
  }

  for (int row = goalRow - 1; row <= goalRow + 1; ++row) {
    goalCells.emplace_back(GridCell(goalCol - 2, row));
    goalCells.emplace_back(GridCell(goalCol + 2, row));
  }

  std::vector<GridCell> whiteCells;
  for (int row = goalRow - 1; row <= goalRow + 1; ++row) {
    for (int col = goalCol - 1; col <= goalCol + 1; ++col) {
      whiteCells.emplace_back(GridCell(col, row));
    }
  }

  markCells(whiteCells, Pixel(255));

  goalCells.emplace_back(goal);
  markCells(goalCells, Pixel(70));
}

}
