
//
// Created by allen on 7/8/19.
//

#ifndef WAVEFRONT_DEBUGGRID_H
#define WAVEFRONT_DEBUGGRID_H

#include "OccGrid.h"
#include "Pixel.h"


class DebugGrid {
public:
  DebugGrid();
  DebugGrid(const std::string &filename, double mapScale);
  virtual ~DebugGrid() = default;
  void outputGrid(const std::string &filename);
  void markWaves(const OccGrid &waveGrid, const std::string &waveColor);
  void markCell(const GridCell &cell, const Pixel &color);
  void markCells(const std::vector<GridCell> &cells, const Pixel &color);
  void markCells(const std::list<GridCell> &cells, const Pixel &color);
  void markStart(const GridCell &start);
  void markGoal(const GridCell &goal);


private:
  std::vector<std::vector<Pixel> > colorGrid;

};

#endif //WAVEFRONT_DEBUGGRID_H
