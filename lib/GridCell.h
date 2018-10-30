/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: GridCell.h
 */

#ifndef WAVEFRONT_GRIDCELL_H
#define WAVEFRONT_GRIDCELL_H

#include "Point.h"

/**
 * A GridCell stores the occupancy grid coordinates of a cell as a row/col pair.
 */

class GridCell {

public:
    GridCell();
    GridCell(int row, int col);
    virtual ~GridCell() = default;
    int getRow();
    void setRow(int row);
    int getCol();
    void setCol(int col);
    bool equals(GridCell cell);
    void printCell();

protected:

private:
    int row;
    int col;
};

#endif //WAVEFRONT_GRIDCELL_H
