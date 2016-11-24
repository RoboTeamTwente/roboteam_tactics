#pragma once

#include <set>
#include <string>
#include <vector>

namespace rtt {

class RobotDealer {
public:
    static void initialize_robots(int keeper, std::vector<int> ids);
    static std::vector<int> get_available_robots();
    static int get_keeper();
    static bool get_keeper_available();
    static void claim_robot(int id);
    static void release_robot(int id);
    static void claim_robots(std::vector<int> ids);
    static void release_robots(std::vector<int> ids);

private:
    static std::set<int> taken_robots;
    static std::set<int> available_robots;
    static int keeper;
    static bool keeper_available;
} ;

} // rtt
