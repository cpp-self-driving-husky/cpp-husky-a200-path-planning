//
// Created by allen on 7/8/19.
//

#include "pathplanning/DebugGrid.h"


DebugGrid::DebugGrid() {
  colorGrid.resize(GRID_HEIGHT);
  Pixel fill = Pixel(255);
  for (auto &row : colorGrid) {
    row.resize(GRID_WIDTH, fill);
  }
}


DebugGrid::DebugGrid(const std::string &filename, double mapScale) {
  colorGrid.resize(GRID_HEIGHT);
  Pixel fill = Pixel(255);
  for (auto &row : colorGrid) {
    row.resize(GRID_WIDTH, fill);
  }

  long inRow, inCol;
  long inputWidth, inputHeight, maxVal;

  std::string fileFormat;
  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ifstream::in);

  /* Read past first line */
  ifs >> fileFormat;
  /* Read in inputWidth, inputHeight, maxVal */
  ifs >> inputWidth >> inputHeight >> maxVal;

  Pixel currGridPixel;
  long gridRow, gridCol;

  if (fileFormat == "P5") {
    char nextChar;
    for (inRow = 0; inRow < inputHeight; ++inRow) {
      for (inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextChar;
        gridRow = static_cast<long>(inRow / mapScale);
        gridCol = static_cast<long>(inCol / mapScale);
        currGridPixel = colorGrid[gridRow][gridCol];
        if (!currGridPixel.isBlack()) {
          colorGrid[gridRow][gridCol] = Pixel(nextChar);
        }
      }
    }
  }

  if (fileFormat == "P2") {
    long nextLong;
    for (inRow = 0; inRow < inputHeight; ++inRow) {
      for (inCol = 0; inCol < inputWidth; ++inCol) {
        ifs >> nextLong;
        gridRow = static_cast<long>(inRow / mapScale);
        gridCol = static_cast<long>(inCol / mapScale);
        currGridPixel = colorGrid[gridRow][gridCol];
        if (!currGridPixel.isBlack()) {
          colorGrid[gridRow][gridCol] = Pixel(nextLong);
        }
      }
    }
  }
  ifs.close();
}


void DebugGrid::outputGrid(const std::string &filename) {
  std::ofstream outFile(filename.c_str(), std::ofstream::out);
  outFile << "P3" << std::endl;
  outFile << GRID_WIDTH << " " << GRID_HEIGHT << std::endl << 255 << std::endl;

  for (long row = 0; row < GRID_HEIGHT; ++row) {
    for (long col = 0; col < GRID_WIDTH; ++col) {
      outFile << colorGrid[row][col].toString() << " ";
    }
  }
  outFile.close();
}


void DebugGrid::markWaves(const OccGrid &waveGrid, const std::string &waveColor) {
  long maxDistance = waveGrid.findMaxDistance();
  long tintColor;

  Pixel white = Pixel(255);
  Pixel black = Pixel(0);

  for (long row = 0; row < GRID_HEIGHT; ++row) {
    for (long col = 0; col < GRID_WIDTH; ++col) {
      if (waveGrid.get(col, row) > 1) {
        tintColor = static_cast<long>(((std::trunc(10.0 * (maxDistance - waveGrid.get(col, row)) / maxDistance)) / 10.0) * 255);
        if (waveColor == "R") {
          colorGrid[row][col] = Pixel(255, tintColor, tintColor);
        } else if (waveColor == "G") {
          colorGrid[row][col] = Pixel(tintColor, 255, tintColor);
        } else if (waveColor == "B") {
          colorGrid[row][col] = Pixel(tintColor, tintColor, 255);
        }
      }
    }
  }
}


void DebugGrid::markCell(const GridCell &cell, const Pixel &color) {
  long row {cell.getRow()};
  long col {cell.getCol()};
  if ((row >= 0) && (row < GRID_HEIGHT) && (col >= 0) && (col < GRID_WIDTH)) {
    colorGrid[row][col] = color;
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
  long startRow = start.getRow();
  long startCol = start.getCol();
  
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
  
  markCells(startCells, Pixel(0, 0, 255));
  
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
  long goalRow = goal.getRow();
  long goalCol = goal.getCol();

  std::vector<GridCell> goalCells;

  for (long col = goalCol - 2; col <= goalCol + 2; ++col) {
    goalCells.emplace_back(GridCell(col, goalRow - 2));
    goalCells.emplace_back(GridCell(col, goalRow + 2));
  }

  for (long row = goalRow - 1; row <= goalRow + 1; ++row) {
    goalCells.emplace_back(GridCell(goalCol - 2, row));
    goalCells.emplace_back(GridCell(goalCol + 2, row));
  }

  std::vector<GridCell> whiteCells;
  for (long row = goalRow - 1; row <= goalRow + 1; ++row) {
    for (long col = goalCol - 1; col <= goalCol + 1; ++col) {
      whiteCells.emplace_back(GridCell(col, row));
    }
  }

  markCells(whiteCells, Pixel(255));

  goalCells.emplace_back(goal);
  markCells(goalCells, Pixel(0, 0, 255));
}
