//
// Created by allen on 7/8/19.
//

#ifndef WAVEFRONT_PIXEL_H
#define WAVEFRONT_PIXEL_H

#include <string>
#include <vector>


class Pixel {
public:
  Pixel();
  explicit Pixel(int grey);
  Pixel(int r, int g, int b);
  virtual ~Pixel();
  std::string toString();
  bool isBlack();
  bool isWhite();

private:
  std::vector<int> rgb;
};


#endif //WAVEFRONT_PIXEL_H
