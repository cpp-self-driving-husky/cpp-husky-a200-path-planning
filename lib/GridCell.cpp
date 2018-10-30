/**
 * project: CATE
 * team: Behavioral (Path Planning)
 * author: Allen Kim
 * file: GridCell.cpp
 */

#include <iostream>
#include "GridCell.h"


GridCell::GridCell() : row(0), col(0) {
}

GridCell::GridCell(int row, int col) : row(row), col(col) {
}

int GridCell::getRow() {
    return row;
}

void GridCell::setRow(int row) {
    GridCell::row = row;
}

int GridCell::getCol() {
    return col;
}

void GridCell::setCol(int col) {
    GridCell::col = col;
}

bool GridCell::equals(GridCell cell) {
    return (col == cell.col && row == cell.row);
}

void GridCell::printCell() {
    std::cout << getCol() << ", " << getRow() << std::endl;
}
