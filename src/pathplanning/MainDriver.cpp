//
// Created by ak on 7/22/19.
//

#include "pathplanning/MainDriver.h"

namespace pathplanner {

//void WaveNav::loadTestXY(std::string filename) {
//  std::ifstream ifs;
//  std::string name;
//  double x, y;
//  Point destination;
//
//  ifs.open(filename.c_str(), std::ifstream::in);
//  if (ifs) {
//    while (ifs >> name) {
//      ifs >> x >> y;
//      destination = Point(x, y);
//      testXY.insert({name, destination});
//    }
//    ifs.close();
//  }
//}


WaveNav::PathPlannerOutput findAnyPath(WaveNav &myNav,
                                       const std::string &waveOption,
                                       Point &startPoint,
                                       Point &goalPoint) {
  GridCell startCell = pointToCell(startPoint);
  GridCell goalCell = pointToCell(goalPoint);
  return myNav.planPath(startCell, goalCell, waveOption);
}


void printOutput(const WaveNav::PathPlannerOutput &out) {
  std::cout << "  " << out.waveType << ": \n    CPU time = " << out.cpuTime
            << "\n    cells visited = " << out.numCellsVisited
            << "\n    initial path length = " << out.initialPathLength
            << "\n    smoothed path length = " << out.smoothPathLength << std::endl;
}

}  // namespace

int main() {
  boost::filesystem::path p {"output/"};
  boost::filesystem::create_directory(p);

  pathplanner::Point topLeft {8, 6};
  pathplanner::Point bottomRight {185, 183};
  pathplanner::Point bottomLeft {12, 175};
  pathplanner::Point midRight {180, 90};
  pathplanner::Point midLeft {10, 90};
  pathplanner::Point centerLeft {70, 65};
  pathplanner::Point middle {90, 85};
  pathplanner::Point test72 {30, 50};

  std::cout << "\nmap test\n";
  pathplanner::WaveNav myNav("../bitmaps/2dmap-02.pbm", "output/2dmap_OFWF");
  pathplanner::WaveNav::PathPlannerOutput pathmap1o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(pathmap1o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-02.pbm", "output/2dmap_Basic");
  pathplanner::WaveNav::PathPlannerOutput pathmap1b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(pathmap1b);
  std::cout << "-----------\n";

  std::cout << "\ntest1\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path11o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path11o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path11b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path11b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.2_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path12o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
  printOutput(path12o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test1.pbm", "output/test1.2_Basic");
  pathplanner::WaveNav::PathPlannerOutput path12b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
  printOutput(path12b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";

  std::cout << "\ntest2\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path21o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path21o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path21b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path21b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.2_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path22o = findAnyPath(myNav, "OFWF", midLeft, midRight);
  printOutput(path22o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.2_Basic");
  pathplanner::WaveNav::PathPlannerOutput path22b = findAnyPath(myNav, "Basic", midLeft, midRight);
  printOutput(path22b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.3_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path23o = findAnyPath(myNav, "OFWF", midRight, midLeft);
  printOutput(path23o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test2.pbm", "output/test2.3_Basic");
  pathplanner::WaveNav::PathPlannerOutput path23b = findAnyPath(myNav, "Basic", midRight, midLeft);
  printOutput(path23b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";

  pathplanner::Point upperMiddle {60, 30};
  std::cout << "\ntest3\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path31o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path31o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path31b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path31b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.2_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path32o = findAnyPath(myNav, "OFWF", topLeft, upperMiddle);
  printOutput(path32o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test3.pbm", "output/test3.2_Basic");
  pathplanner::WaveNav::PathPlannerOutput path32b = findAnyPath(myNav, "Basic", topLeft, upperMiddle);
  printOutput(path32b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";

  std::cout << "\ntest4\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path41o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path41o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path41b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path41b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.2_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path42o = findAnyPath(myNav, "OFWF", midLeft, midRight);
  printOutput(path42o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.2_Basic");
  pathplanner::WaveNav::PathPlannerOutput path42b = findAnyPath(myNav, "Basic", midLeft, midRight);
  printOutput(path42b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.3_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path43o = findAnyPath(myNav, "OFWF", midLeft, middle);
  printOutput(path43o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.3_Basic");
  pathplanner::WaveNav::PathPlannerOutput path43b = findAnyPath(myNav, "Basic", midLeft, middle);
  printOutput(path43b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.4_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path44o = findAnyPath(myNav, "OFWF", middle, midLeft);
  printOutput(path44o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test4.pbm", "output/test4.4_Basic");
  pathplanner::WaveNav::PathPlannerOutput path44b = findAnyPath(myNav, "Basic", middle, midLeft);
  printOutput(path44b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";

  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.1_OFWF");
  std::cout << "\ntest5\n";
//  pathplanner::WaveNav myNav("../bitmaps/2dmap-test5.pbm", "output/test5.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path51o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path51o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path51b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path51b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.2_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path52o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
  printOutput(path52o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.2_Basic");
  pathplanner::WaveNav::PathPlannerOutput path52b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
  printOutput(path52b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.3_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path53o = findAnyPath(myNav, "OFWF", bottomLeft, middle);
  printOutput(path53o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test5.pbm", "output/test5.3_Basic");
  pathplanner::WaveNav::PathPlannerOutput path53b = findAnyPath(myNav, "Basic", bottomLeft, middle);
  printOutput(path53b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";

  std::cout << "\ntest6\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path61o = findAnyPath(myNav, "OFWF", topLeft, bottomRight);
  printOutput(path61o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path61b = findAnyPath(myNav, "Basic", topLeft, bottomRight);
  printOutput(path61b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.2_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path62o = findAnyPath(myNav, "OFWF", bottomLeft, midRight);
  printOutput(path62o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.2_Basic");
  pathplanner::WaveNav::PathPlannerOutput path62b = findAnyPath(myNav, "Basic", bottomLeft, midRight);
  printOutput(path62b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.3_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path63o = findAnyPath(myNav, "OFWF", bottomLeft, centerLeft);
  printOutput(path63o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.3_Basic");
  pathplanner::WaveNav::PathPlannerOutput path63b = findAnyPath(myNav, "Basic", bottomLeft, centerLeft);
  printOutput(path63b);
  std::cout << "-----------\n";
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.4_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path64o = findAnyPath(myNav, "OFWF", topLeft, centerLeft);
  printOutput(path64o);
  myNav = pathplanner::WaveNav("../bitmaps/2dmap-test6.pbm", "output/test6.4_Basic");
  pathplanner::WaveNav::PathPlannerOutput path64b = findAnyPath(myNav, "Basic", topLeft, centerLeft);
  printOutput(path64b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";

  std::cout << "\ntest7\n";
  myNav = pathplanner::WaveNav("../bitmaps/test7.pbm", "output/test7.1_OFWF");
  pathplanner::WaveNav::PathPlannerOutput path71o = findAnyPath(myNav, "OFWF", topLeft, test72);
  printOutput(path71o);
  myNav = pathplanner::WaveNav("../bitmaps/test7.pbm", "output/test7.1_Basic");
  pathplanner::WaveNav::PathPlannerOutput path71b = findAnyPath(myNav, "Basic", topLeft, test72);
  printOutput(path71b);
  std::cout << "-----------\n";
  std::cout << "-----------\n";
  return 0;
}


