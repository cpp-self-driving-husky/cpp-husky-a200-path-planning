//
// Created by allen on 7/8/19.
//

#include "../../include/pathplanning/Pixel.h"

Pixel::Pixel() {
  rgb.resize(3, 0);
}


Pixel::Pixel(int grey) {
  rgb.resize(3, grey);
}


Pixel::Pixel(int r, int g, int b) {
  rgb.reserve(3);
  rgb.emplace_back(r);
  rgb.emplace_back(g);
  rgb.emplace_back(b);

}


Pixel::~Pixel() = default;

std::string Pixel::toString() {
  return std::to_string(rgb[0]) + " " + std::to_string(rgb[1]) + " " + std::to_string(rgb[2]);
}


bool Pixel::isBlack() {
  return ((rgb[0] == 0) && (rgb[1] == 0) && (rgb[2] == 0));
}


bool Pixel::isWhite() {
  return ((rgb[0] == 255) && (rgb[1] == 255) && (rgb[2] == 255));
}
