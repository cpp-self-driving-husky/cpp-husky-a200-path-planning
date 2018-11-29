// Allen Kim | Mar. 14, 2017
// CS 521 (Winter, 2017)
// Project 4: Path Planning & Navigation
// proj4.cc

#include <libplayerc++/playerc++.h>
#include <WavefrontNavigator.h>


void printIntro() {
    std::cout << ".--+*%$%*+-+*%$%*+-+*%$-+*%$%*+-+*%$%*+-+*%$-+*%$%*+-+*%$%*+--.\n";
    std::cout << "!                                                             !\n";
    std::cout << ":                     CS 521 - Project 4                      :\n";
    std::cout << ".                Path Planning & Navigation                   .\n";
    std::cout << ".                                                             .\n";
    std::cout << ":                    Allen Kim | 03/14/17                     :\n";
    std::cout << "!                                                             !\n";
    std::cout << ".--+*%$%*+-+*%$%*+-+*%$-+*%$%*+-+*%$%*+-+*%$-+*%$%*+-+*%$%*+--.\n" << std::endl;
}


int main(int argc, char *argv[]) {
    printIntro();
    std::cout << "    --+*%$[ Press <Enter> to start the simulation. ]$%*+--" << std::endl;
    std::cin.ignore();
    try {
        // setup robot and proxies
        std::cout << "\n --+*%$ Initializing..." << std::endl;
        std::cout << "   +*%$ Connecting to Player Server... " << std::endl;
        PlayerCc::PlayerClient robot("localhost");
        robot.SetReplaceRule(true);
        PlayerCc::Position2dProxy pp(&robot, 0);
        PlayerCc::LaserProxy laserProxy(&robot, 0);
        std::cout << "      $ DONE!" << std::endl;

        std::cout << "   +*%$ Starting up robot control modules..." << std::endl;
        // enable motors
        pp.SetMotorEnable(1);

        // request geometries/configs
        pp.RequestGeom();
//        laserProxy.RequestGeom();
//        laserProxy.RequestConfigure();
        robot.Read();

        // initialize robot control modules
        ACT simACT(&pp);
        WavefrontNavigator simNav("../bitmaps/hospital_section.pnm");
        Pilot simPilot(&simACT, &simNav, &robot, &pp, &laserProxy);
        simNav.setMyPilot(&simPilot);
        simNav.setMyPP(&pp);
        robot.Read();

        std::cout << "      $ DONE!" << std::endl;
        std::cout << "\n --+*%$ Path Planning & Navigation Tester" << std::endl;

        char userInput;
        do  {
            robot.Read();
            simNav.processUserWaypoint();

            std::cout << "  -+*%$ Send robot to another location? (y/n) > ";
            std::cin >> userInput;
        } while (userInput != 'n');

        std::cout << "\n --+*%$ Thank you, bye!" << std::endl;
        exit(0);
    } catch (PlayerCc::PlayerError &e) {
        std::cerr << e << std::endl;
        return -1;
    }
}


