#pragma once

#include <vector>

namespace rtt{

    class RobotsToDefendFinder{
    public:
        static std::vector<int> GetRobotsToDefend(double minDangerScore, bool excludeKicker);
    };

}