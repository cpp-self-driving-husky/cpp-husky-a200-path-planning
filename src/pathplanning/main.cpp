/**
 * project: CATE
 *    team: Behavioral (Path Planning)
 *  author: Allen Kim
 *    file: main.cpp
 */

/**
 * Example of how to run a path search. This version is run from the command line with
 * 4 parameters: source-latitude, source-longitude, target-latitude, target-longitude.
 *
 * Outputs a list of GPS coordinates along the optimal path.
 */

#include <iostream>
#include "../../include/pathplanning/WaveNav.h"

namespace pp = pathplanner;

int main(int argc, char *argv[]) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " source-latitude source-longitude target-latitude target-longitude\n";
    return 1;
  }

  std::string mapFile = "../bitmaps/grown_map_scale_2.pbm";
  std::string outputPathStub = "output/run_map2_OFWF";

  double source_latitude = std::atof(argv[1]);
  double source_longitude = std::atof(argv[2]);
  double target_latitude = std::atof(argv[3]);
  double target_longitude = std::atof(argv[4]);
  pp::GPS source(source_longitude, source_latitude);
  pp::GPS target(target_longitude, target_latitude);

  pp::WaveNav myNav = pp::WaveNav(mapFile, outputPathStub);
  myNav.planPath(source, target, "OFWF", true);

  for (const auto &wayCell : myNav.getSmoothedPath()) {
    pp::GPS cellGPS = pp::cellToGPS(wayCell);
    std::cout << std::fixed << std::setprecision(6) \
        << "(" << cellGPS.getLatitude() << ", " \
        << cellGPS.getLongitude() << ")" \
        << std::endl;
  }
  return 0;
}
