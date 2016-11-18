#pragma once

#include <set>
#include <string>
#include <vector>

namespace rtt {

class RobotDealer {
    public:
    static std::set<int> taken_robots;
    static std::set<int> available_robots;

    static void initialize_robots(std::vector<int> ids);
    static std::vector<int> get_available_robots();
    static void claim_robot(int id);
    static void release_robot(int id);
    static void claim_robots(std::vector<int> ids);
    static void release_robots(std::vector<int> ids);
} ;

} // rtt
