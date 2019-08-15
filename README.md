# cpp-husky-a200-path-planning
Wavefront propagation path-planning for university quad

Run cmake with top-level CMakeList.txt

Once built, run program with the following command

     ./wavefront <source-latitude> <source-longitude> <target-latitude> <target-longitude>

It will output to cout a list of GPS coordinates along the optimal path. Also, a map will be displayed showing initial and smooth paths, waypoints, and which nodes were expanded.
