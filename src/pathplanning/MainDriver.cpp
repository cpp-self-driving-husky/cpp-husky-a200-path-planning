//
// Created by ak on 7/22/19.
//

#include "pathplanning/MainDriver.h"

namespace pathplanner {



void loadDestinations(const std::string &filename, std::unordered_map<std::string, GridCell> *destinations) {
  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ifstream::in);
  if (ifs) {
    std::string name;
    double latitude {0.0};
    double longitude {0.0};
    GridCell destination;
    while (ifs >> name) {
      ifs >> latitude >> longitude;
      GPS gpsCoords {longitude, latitude};
      destination = gpsToCell(gpsCoords);
      destinations->insert({name, destination});
    }
  }
  ifs.close();
}


WaveNav::ppOutput findAnyPath(WaveNav &myNav,
                              const std::string &waveOption,
                              GridCell &start,
                              GridCell &goal,
                              int debugLevel) {
  return myNav.planPath(start, goal, waveOption, debugLevel);
}


void printOutput(const WaveNav::ppOutput &out) {
  std::cout << out.waveType_ << "\t"
            << std::setw(12) << out.initialPathLength_
            << std::setw(12) << out.smoothPathLength_
            << std::setw(9) << out.numCellsVisited_;
}


void runTest(const std::string &mapFile,
             const std::string &waveType,
             const std::string &outputPathStub,
             const std::string &startKey,
             const std::string &goalKey,
             std::unordered_map <std::string, GridCell> &POIs,
             int trials,
             int debugLevel) {

  WaveNav myNav;
  WaveNav::ppOutput path;
  std::vector<int> cpuTimes;

  std::cout << startKey << " to " << goalKey << "  ";

  for (int i = 0; i < trials; ++i) {
    myNav = WaveNav(mapFile, outputPathStub);
    path = findAnyPath(myNav, waveType, POIs[startKey], POIs[goalKey], debugLevel);
    cpuTimes.emplace_back(path.cpuTime_);
    if (i == trials - 1) {
      printOutput(path);
    }
  }
  double averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
  std::cout << std::setw(9) << averageCpuTime << "\n";
  cpuTimes.clear();
}

}  // namespace

namespace pp = pathplanner;



int main() {
//  boost::filesystem::path p {"output/"};
//  boost::filesystem::create_directory(p);

  const int TRIALS = 4;
  const int DEBUG = 1;
  

  std::unordered_map<std::string, pp::GridCell> POIs;
  pp::loadDestinations("../dest_coordinates.txt", &POIs);

  std::string startDest, goalDest, outPath;
  pp::GridCell startCell, goalCell;
  for (int i = 1; i <= 9; ++i) {
    for (int j = 1; j <= 9; ++j) {
      startDest = "dest0" + std::to_string(i);
      goalDest = "dest0" + std::to_string(j);
      if (startDest != goalDest) {
        outPath = "output2/";
        outPath += startDest.substr(4, 2);
        outPath += "-";
        outPath += goalDest.substr(4, 2);
        outPath += "_OFWF";
        pp::runTest("../bitmaps/2dmap-01.pbm", "OFWF", outPath, startDest, goalDest, POIs, TRIALS, DEBUG);

        outPath = "output2/";
        outPath += startDest.substr(4, 2);
        outPath += "-";
        outPath += goalDest.substr(4, 2);
        outPath += "_Basic";
        pp::runTest("../bitmaps/2dmap-01.pbm", "Basic", outPath, startDest, goalDest, POIs, TRIALS, DEBUG);
      }
    }
  }

  pp::Point bottomRight {185, 183};
  pp::Point topLeft {8, 6};
  pp::Point bottomLeft {12, 175};
  pp::Point midRight {180, 90};
  pp::Point midLeft {10, 90};
  pp::Point upperMiddle {60, 30};
  pp::Point centerLeft {70, 65};
  pp::Point middle {90, 85};
  pp::Point test72 {30, 50};
  pp::Point topRight {170, 6};


  pp::GridCell br = pp::pointToCell(bottomRight);
  pp::GridCell tl = pp::pointToCell(topLeft);
  pp::GridCell bl = pp::pointToCell(bottomLeft);
  pp::GridCell mr = pp::pointToCell(midRight);
  pp::GridCell ml = pp::pointToCell(midLeft);
  pp::GridCell um = pp::pointToCell(upperMiddle);
  pp::GridCell cl = pp::pointToCell(centerLeft);
  pp::GridCell m = pp::pointToCell(middle);
  pp::GridCell t72 = pp::pointToCell(test72);
  pp::GridCell tr = pp::pointToCell(topRight);

  POIs.insert({"br", br});
  POIs.insert({"tl", tl});
  POIs.insert({"bl", bl});
  POIs.insert({"mr", mr});
  POIs.insert({"ml", ml});
  POIs.insert({"um", um});
  POIs.insert({"cl", cl});
  POIs.insert({"m", m});
  POIs.insert({"t72",t72});
  POIs.insert({"tr",tr});




//  for (int i = 1; i <= 7; ++i) {
//    std::string fname = "../bitmaps/testmap" + std::to_string(i) + ".pbm";
//
//  }

  std::cout << "\ntestmap8\n";
  pp::runTest("../bitmaps/testmap8.pbm", "OFWF",  "output/map8.1_OFWF",  "m", "um", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap8.pbm", "Basic", "output/map8.1_Basic", "m", "um", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap8.pbm", "OFWF",  "output/map8.2_OFWF",  "m", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap8.pbm", "Basic", "output/map8.2_Basic", "m", "mr", POIs, TRIALS, DEBUG);



  std::cout << "\ntestmap1\n";
  pp::runTest("../bitmaps/testmap1.pbm", "OFWF",  "output/map1.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap1.pbm", "Basic", "output/map1.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap1.pbm", "OFWF",  "output/map1.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap1.pbm", "Basic", "output/map1.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap1.pbm", "OFWF",  "output/map1.2_OFWF",  "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap1.pbm", "Basic", "output/map1.2_Basic", "bl", "mr", POIs, TRIALS, DEBUG);

  std::cout << "\ntestmap2\n";
  pp::runTest("../bitmaps/testmap2.pbm", "OFWF",  "output/map2.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "Basic", "output/map2.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "OFWF",  "output/map2.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "Basic", "output/map2.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "OFWF",  "output/map2.2_OFWF",  "ml", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "Basic", "output/map2.2_Basic", "ml", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "OFWF",  "output/map2.3_OFWF",  "mr", "ml", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap2.pbm", "Basic", "output/map2.3_Basic", "mr", "ml", POIs, TRIALS, DEBUG);

  std::cout << "\ntestmap3\n";
  pp::runTest("../bitmaps/testmap3.pbm", "OFWF",  "output/map3.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap3.pbm", "Basic", "output/map3.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap3.pbm", "OFWF",  "output/map3.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap3.pbm", "Basic", "output/map3.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap3.pbm", "OFWF",  "output/map3.2_OFWF",  "tl", "um", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap3.pbm", "Basic", "output/map3.2_Basic", "tl", "um", POIs, TRIALS, DEBUG);

  std::cout << "\ntestmap4\n";
  pp::runTest("../bitmaps/testmap4.pbm", "OFWF",  "output/map4.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "Basic", "output/map4.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "OFWF",  "output/map4.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "Basic", "output/map4.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "OFWF",  "output/map4.2_OFWF",  "ml", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "Basic", "output/map4.2_Basic", "ml", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "OFWF",  "output/map4.3_OFWF",  "ml", "m",  POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "Basic", "output/map4.3_Basic", "ml", "m",  POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "OFWF",  "output/map4.4_OFWF",  "m",  "ml", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap4.pbm", "Basic", "output/map4.4_Basic", "m",  "ml", POIs, TRIALS, DEBUG);

  std::cout << "\ntestmap5\n";
  pp::runTest("../bitmaps/testmap5.pbm", "OFWF",  "output/map5.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "Basic", "output/map5.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "OFWF",  "output/map5.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "Basic", "output/map5.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "OFWF",  "output/map5.2_OFWF",  "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "Basic", "output/map5.2_Basic", "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "OFWF",  "output/map5.3_OFWF",  "bl", "m",  POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap5.pbm", "Basic", "output/map5.3_Basic", "bl", "m",  POIs, TRIALS, DEBUG);

  std::cout << "\ntestmap6\n";
  pp::runTest("../bitmaps/testmap6.pbm", "OFWF",  "output/map6.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "Basic", "output/map6.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "OFWF",  "output/map6.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "Basic", "output/map6.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "OFWF",  "output/map6.2_OFWF",  "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "Basic", "output/map6.2_Basic", "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "OFWF",  "output/map6.3_OFWF",  "bl", "m",  POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "Basic", "output/map6.3_Basic", "bl", "m",  POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "OFWF",  "output/map6.4_OFWF",  "tl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "Basic", "output/map6.4_Basic", "tl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "OFWF",  "output/map6.5_OFWF",  "m",  "um", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/testmap6.pbm", "Basic", "output/map6.5_Basic", "m",  "um", POIs, TRIALS, DEBUG);

  std::cout << "\nbigmaze\n";
  pp::runTest("../bitmaps/mazebig1860.pbm", "OFWF",  "output/big.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "Basic", "output/big.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "OFWF",  "output/big.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "Basic", "output/big.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "OFWF",  "output/big.2_OFWF",  "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "Basic", "output/big.2_Basic", "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "OFWF",  "output/big.3_OFWF",  "bl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "Basic", "output/big.3_Basic", "bl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "OFWF",  "output/big.4_OFWF",  "tl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "Basic", "output/big.4_Basic", "tl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "OFWF",  "output/big.5_OFWF",  "m",  "um", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/mazebig1860.pbm", "Basic", "output/big.5_Basic", "m",  "um", POIs, TRIALS, DEBUG);

  std::cout << "\ntite maze\n";
  pp::runTest("../bitmaps/tightmaze1860.pbm", "OFWF",  "output/maze.0_OFWF",  "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "Basic", "output/maze.0_Basic", "bl", "tr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "OFWF",  "output/maze.1_OFWF",  "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "Basic", "output/maze.1_Basic", "tl", "br", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "OFWF",  "output/maze.2_OFWF",  "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "Basic", "output/maze.2_Basic", "bl", "mr", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "OFWF",  "output/maze.3_OFWF",  "bl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "Basic", "output/maze.3_Basic", "bl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "OFWF",  "output/maze.4_OFWF",  "tl", "cl", POIs, TRIALS, DEBUG);
  pp::runTest("../bitmaps/tightmaze1860.pbm", "Basic", "output/maze.4_Basic", "tl", "cl", POIs, TRIALS, DEBUG);


//  std::cout << "\ntest7\n";
//  myNav = pp::WaveNav("../bitmaps/test7.pbm", "output/test7.1_OFWF");
//  pp::WaveNav::ppOutput path71o = findAnyPath(myNav, "OFWF", topLeft, test72);
//  printOutput(path71o);
//  myNav = pp::WaveNav("../bitmaps/test7.pbm", "output/test7.1_Basic");
//  pp::WaveNav::ppOutput path71b = findAnyPath(myNav, "Basic", topLeft, test72);
//  printOutput(path71b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//  return 0;
}


