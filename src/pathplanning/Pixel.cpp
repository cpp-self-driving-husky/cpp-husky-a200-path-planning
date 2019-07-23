//
// Created by allen on 7/8/19.
//

#include "pathplanning/Pixel.h"

namespace pathplanner {

Pixel::Pixel() {
  rgb_.resize(3, 0);
}


Pixel::Pixel(int grey) {
  rgb_.resize(3, grey);
}


Pixel::Pixel(int r, int g, int b) {
  rgb_.reserve(3);
  rgb_.emplace_back(r);
  rgb_.emplace_back(g);
  rgb_.emplace_back(b);
}


Pixel::~Pixel() = default;


std::string Pixel::toString() {
  return std::to_string(rgb_[0]) + " " + std::to_string(rgb_[1]) + " " + std::to_string(rgb_[2]);
}


bool Pixel::isBlack() {
  return ((rgb_[0] == 0) && (rgb_[1] == 0) && (rgb_[2] == 0));
}


bool Pixel::isWhite() {
  return ((rgb_[0] == 255) && (rgb_[1] == 255) && (rgb_[2] == 255));
}

}
