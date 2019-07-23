//
// Created by ak on 7/22/19.
//

#include "pathplanning/MainDriver.h"

namespace pathplanner {

void loadDestinations(const std::string &filename, std::unordered_map<std::string, Point> *destinations) {
  std::ifstream ifs;
  ifs.open(filename.c_str(), std::ifstream::in);
  if (ifs) {
    std::string name;
    double latitude {0.0};
    double longitude {0.0};
    Point destination;
    while (ifs >> name) {
      ifs >> latitude >> longitude;
      destination = Point(longitudeToX(longitude), latitudeToY(latitude));
      destinations->insert({name, destination});
    }
  }
  ifs.close();
}


WaveNav::ppOutput findAnyPath(WaveNav &myNav,
                                       const std::string &waveOption,
                                       Point &startPoint,
                                       Point &goalPoint) {
  GridCell startCell = pointToCell(startPoint);
  GridCell goalCell = pointToCell(goalPoint);
  return myNav.planPath(startCell, goalCell, waveOption);
}


void printOutput(const WaveNav::ppOutput &out) {
  std::cout << "  " << out.waveType
            << ": cells visited = " << out.numCellsVisited
            << ", initial path length = " << out.initialPathLength
            << ", smoothed path length = " << out.smoothPathLength << std::endl;
}


void runTest(const std::string &start, const std::string &goal,
             std::unordered_map<std::string, Point> &points) {
  std::string startStr = "dest" + start;
  std::string goalStr = "dest" + goal;
  std::string outputPath = "output/" + start + "-" + goal + "_";

  std::cout << "MAP TEST - " << startStr << " to " << goalStr << std::endl;
  WaveNav myNav("../bitmaps/2dmap-01.pbm", outputPath + "OFWF");
  WaveNav::ppOutput path = findAnyPath(myNav, "OFWF", points[startStr], points[goalStr]);
  std::vector<int> cpuTimes;
  cpuTimes.reserve(10);
  cpuTimes.emplace_back(path.cpuTime);

  for (int i = 1; i < 10; ++i) {
    myNav = WaveNav("../bitmaps/2dmap-01.pbm", outputPath + "OFWF");
    path = findAnyPath(myNav, "OFWF", points[startStr], points[goalStr]);
    cpuTimes.emplace_back(path.cpuTime);
  }
  printOutput(path);
  double averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
  std::cout << "        avg CPU time: " << averageCpuTime << " microseconds\n";
  cpuTimes.clear();

  for (int i = 0; i < 10; ++i) {
    myNav = WaveNav("../bitmaps/2dmap-01.pbm", outputPath + "Basic");
    path = findAnyPath(myNav, "Basic", points[startStr], points[goalStr]);
    cpuTimes.emplace_back(path.cpuTime);
  }
  printOutput(path);
  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
  std::cout << "         avg CPU time: " << averageCpuTime << " microseconds\n";
  std::cout << "-----------\n";
  cpuTimes.clear();
}

}  // namespace

namespace pp = pathplanner;

int main() {
  boost::filesystem::path p {"output/"};
  boost::filesystem::create_directory(p);

  std::unordered_map<std::string, pp::Point> POIs;
  pp::loadDestinations("../dest_coordinates.txt", &POIs);

  std::string startDest, goalDest;
  for (int i = 1; i <= 9; ++i) {
    for (int j = 1; j <= 9; ++j) {
      startDest = "0" + std::to_string(i);
      goalDest = "0" + std::to_string(j);

      if (startDest != goalDest) {
        pp::runTest(startDest, goalDest, POIs);
      }
    }
  }

//  pp::runTest("01", "02", POIs);
//  pp::runTest("01", "03", POIs);
//  pp::runTest("01", "04", POIs);
//  pp::runTest("01", "05", POIs);
//  pp::runTest("04", "06", POIs);
//  pp::runTest("04", "07", POIs);
//  pp::runTest("04", "08", POIs);
//  pp::runTest("04", "09", POIs);
//  pp::runTest("05", "07", POIs);
//  pp::runTest("05", "02", POIs);
//  pp::runTest("05", "06", POIs);

  // run test 01_02 10 times and store CPU times
//  std::cout << "MAP TEST - dest01 to dest02\n";
//  pp::WaveNav myNav("../bitmaps/2dmap-01.pbm", "output/01-02_OFWF");
//  pp::WaveNav::ppOutput path = findAnyPath(myNav, "OFWF", POIs["dest01"], POIs["dest02"]);
//  std::vector<int> cpuTimes;
//  cpuTimes.reserve(10);
//  cpuTimes.emplace_back(path.cpuTime);
//  pp::printOutput(path);
//
//  for (int i = 1; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/01-02_OFWF");
//    path = findAnyPath(myNav, "OFWF", POIs["dest01"], POIs["dest02"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  double averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "  OFWF avg CPU time: " << averageCpuTime << " microseconds\n";
//  cpuTimes.clear();
//
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/01-02_Basic");
//    path = findAnyPath(myNav, "Basic", POIs["dest01"], POIs["dest02"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "  Basic avg CPU time: " << averageCpuTime << " microseconds\n";
//  std::cout << "-----------\n";
//  cpuTimes.clear();
//
//  std::cout << "MAP TEST - dest01 to dest05\n";
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/01-05_OFWF");
//    path = findAnyPath(myNav, "OFWF", POIs["dest01"], POIs["dest05"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "  OFWF avg CPU time: " << averageCpuTime << " microseconds\n";
//  cpuTimes.clear();
//
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/01-05_Basic");
//    path = findAnyPath(myNav, "Basic", POIs["dest01"], POIs["dest05"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "  Basic avg CPU time: " << averageCpuTime << " microseconds\n";
//  std::cout << "-----------\n";
//  cpuTimes.clear();
//
//  std::cout << "MAP TEST - dest06 to dest03\n";
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/06-03_OFWF");
//    path = findAnyPath(myNav, "OFWF", POIs["dest06"], POIs["dest03"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "  OFWF avg CPU time: " << averageCpuTime << " microseconds\n";
//  cpuTimes.clear();
//
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/06-03_Basic");
//    path = findAnyPath(myNav, "Basic", POIs["dest06"], POIs["dest03"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "  Basic avg CPU time: " << averageCpuTime << " microseconds\n";
//  std::cout << "-----------\n";
//  cpuTimes.clear();
//
//  std::cout << "MAP TEST - dest04 to dest09\n";
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/04-09_OFWF");
//    path = findAnyPath(myNav, "OFWF", POIs["dest04"], POIs["dest09"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "        avg CPU time: " << averageCpuTime << " microseconds\n";
//  cpuTimes.clear();
//
//  for (int i = 0; i < 10; ++i) {
//    myNav = pp::WaveNav("../bitmaps/2dmap-01.pbm", "output/04-09_Basic");
//    path = findAnyPath(myNav, "Basic", POIs["dest04"], POIs["dest09"]);
//    cpuTimes.emplace_back(path.cpuTime);
//  }
//  pp::printOutput(path);
//  averageCpuTime = std::accumulate(cpuTimes.begin(), cpuTimes.end(), 0.0) / cpuTimes.size();
//  std::cout << "         avg CPU time: " << averageCpuTime << " microseconds\n";
//  std::cout << "-----------\n";
//  cpuTimes.clear();





//  pp::Point bottomRight {185, 183};
//  pp::Point topLeft {8, 6};
//  pp::Point bottomLeft {12, 175};
//  pp::Point midRight {180, 90};
//  pp::Point midLeft {10, 90};
//  pp::Point centerLeft {70, 65};
//  pp::Point middle {90, 85};
//  pp::Point test72 {30, 50};
//
//  std::cout << "\nmap test\n";
//  pp::WaveNav myNav("../bitmaps/2dmap-02.pbm", "output/2dmap_OFWF");
//  pp::WaveNav::ppOutput pathmap1o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(pathmap1o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-02.pbm", "output/2dmap_Basic");
//  pp::WaveNav::ppOutput pathmap1b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(pathmap1b);
//  std::cout << "-----------\n";
//
//  std::cout << "\ntest1\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.1_OFWF");
//  pp::WaveNav::ppOutput path11o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(path11o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.1_Basic");
//  pp::WaveNav::ppOutput path11b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(path11b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.2_OFWF");
//  pp::WaveNav::ppOutput path12o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
//  printOutput(path12o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.2_Basic");
//  pp::WaveNav::ppOutput path12b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
//  printOutput(path12b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//
//  std::cout << "\ntest2\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.1_OFWF");
//  pp::WaveNav::ppOutput path21o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(path21o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.1_Basic");
//  pp::WaveNav::ppOutput path21b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(path21b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.2_OFWF");
//  pp::WaveNav::ppOutput path22o = findAnyPath(myNav, "OFWF", midLeft, midRight);
//  printOutput(path22o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.2_Basic");
//  pp::WaveNav::ppOutput path22b = findAnyPath(myNav, "Basic", midLeft, midRight);
//  printOutput(path22b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.3_OFWF");
//  pp::WaveNav::ppOutput path23o = findAnyPath(myNav, "OFWF", midRight, midLeft);
//  printOutput(path23o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.3_Basic");
//  pp::WaveNav::ppOutput path23b = findAnyPath(myNav, "Basic", midRight, midLeft);
//  printOutput(path23b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//
//  pp::Point upperMiddle {60, 30};
//  std::cout << "\ntest3\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.1_OFWF");
//  pp::WaveNav::ppOutput path31o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(path31o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.1_Basic");
//  pp::WaveNav::ppOutput path31b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(path31b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.2_OFWF");
//  pp::WaveNav::ppOutput path32o = findAnyPath(myNav, "OFWF", topLeft, upperMiddle);
//  printOutput(path32o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.2_Basic");
//  pp::WaveNav::ppOutput path32b = findAnyPath(myNav, "Basic", topLeft, upperMiddle);
//  printOutput(path32b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//
//  std::cout << "\ntest4\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.1_OFWF");
//  pp::WaveNav::ppOutput path41o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(path41o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.1_Basic");
//  pp::WaveNav::ppOutput path41b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(path41b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.2_OFWF");
//  pp::WaveNav::ppOutput path42o = findAnyPath(myNav, "OFWF", midLeft, midRight);
//  printOutput(path42o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.2_Basic");
//  pp::WaveNav::ppOutput path42b = findAnyPath(myNav, "Basic", midLeft, midRight);
//  printOutput(path42b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.3_OFWF");
//  pp::WaveNav::ppOutput path43o = findAnyPath(myNav, "OFWF", midLeft, middle);
//  printOutput(path43o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.3_Basic");
//  pp::WaveNav::ppOutput path43b = findAnyPath(myNav, "Basic", midLeft, middle);
//  printOutput(path43b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.4_OFWF");
//  pp::WaveNav::ppOutput path44o = findAnyPath(myNav, "OFWF", middle, midLeft);
//  printOutput(path44o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.4_Basic");
//  pp::WaveNav::ppOutput path44b = findAnyPath(myNav, "Basic", middle, midLeft);
//  printOutput(path44b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//
//  myNav = pp::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.1_OFWF");
//  std::cout << "\ntest5\n";
////  pp::WaveNav myNav("../bitmaps/2dmap-test5.pbm", "output/test5.1_OFWF");
//  pp::WaveNav::ppOutput path51o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(path51o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.1_Basic");
//  pp::WaveNav::ppOutput path51b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(path51b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.2_OFWF");
//  pp::WaveNav::ppOutput path52o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
//  printOutput(path52o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.2_Basic");
//  pp::WaveNav::ppOutput path52b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
//  printOutput(path52b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.3_OFWF");
//  pp::WaveNav::ppOutput path53o = findAnyPath(myNav, "OFWF", bottomLeft, middle);
//  printOutput(path53o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.3_Basic");
//  pp::WaveNav::ppOutput path53b = findAnyPath(myNav, "Basic", bottomLeft, middle);
//  printOutput(path53b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//
//  std::cout << "\ntest6\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.1_OFWF");
//  pp::WaveNav::ppOutput path61o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
//  printOutput(path61o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.1_Basic");
//  pp::WaveNav::ppOutput path61b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
//  printOutput(path61b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.2_OFWF");
//  pp::WaveNav::ppOutput path62o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
//  printOutput(path62o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.2_Basic");
//  pp::WaveNav::ppOutput path62b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
//  printOutput(path62b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.3_OFWF");
//  pp::WaveNav::ppOutput path63o = findAnyPath(myNav, "OFWF", bottomLeft, centerLeft);
//  printOutput(path63o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.3_Basic");
//  pp::WaveNav::ppOutput path63b = findAnyPath(myNav, "Basic", bottomLeft, centerLeft);
//  printOutput(path63b);
//  std::cout << "-----------\n";
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.4_OFWF");
//  pp::WaveNav::ppOutput path64o = findAnyPath(myNav, "OFWF", topLeft, centerLeft);
//  printOutput(path64o);
//  myNav = pp::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.4_Basic");
//  pp::WaveNav::ppOutput path64b = findAnyPath(myNav, "Basic", topLeft, centerLeft);
//  printOutput(path64b);
//  std::cout << "-----------\n";
//  std::cout << "-----------\n";
//
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


