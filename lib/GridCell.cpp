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

int GridCell::getRow() const {
    return row;
}

void GridCell::setRow(int row) {
    GridCell::row = row;
}

int GridCell::getCol() const {
    return col;
}

void GridCell::setCol(int col) {
    GridCell::col = col;
}

bool GridCell::equals(const GridCell cell) const {
    return (col == cell.getCol() && row == cell.getRow());
}

void GridCell::printCell() const {
    std::cout << getCol() << ", " << getRow() << std::endl;
}

bool GridCell::operator==(const GridCell &cell) {
    return (col == cell.getCol() && row == cell.getRow());
}

