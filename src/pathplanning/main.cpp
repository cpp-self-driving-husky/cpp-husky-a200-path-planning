//
// Created by ak on 8/15/19.
//

#include <iostream>
#include "pathplanning/WaveNav.h"


namespace pp = pathplanner;

int main(int argc, char *argv[]) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << "source-latitude source-longitude target-latitude target-longitude" << std::endl;
    return 1;
  }

  std::string mapFile = "../bitmaps/map_scale_2.pbm";
  std::string outputPathStub = "output/run_map2_OFWF";


  double source_latitude = std::atof(argv[1]);
  double source_longitude = std::atof(argv[2]);
  double target_latitude = std::atof(argv[3]);
  double target_longitude = std::atof(argv[4]);
  pp::GPS source(source_longitude, source_latitude);
  pp::GPS target(target_longitude, target_latitude);

  pp::WaveNav myNav;
  pp::WaveNav::ppOutput path;

  myNav = pp::WaveNav(mapFile, outputPathStub);
  path = myNav.planPath(source, target, "OFWF", 0);

//  int precision = std::numeric_limits<double>::max_digits10;

  for (const auto &wayCell : myNav.getSmoothedPath()) {
    pp::GPS cellGPS = pp::cellToGPS(wayCell);
    std::cout << std::fixed << std::setprecision(6) << "(" << cellGPS.getLatitude() << ", " << cellGPS .getLongitude() << ")" << std::endl;
  }
  return 0;
}
