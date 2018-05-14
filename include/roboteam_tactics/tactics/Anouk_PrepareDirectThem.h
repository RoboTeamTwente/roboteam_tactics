#pragma once

#include "roboteam_tactics/tactics/Emiel_Prepare.h"
#include "roboteam_tactics/Parts.h"
#include <boost/uuid/uuid.hpp>
#include <vector>

namespace rtt {

    class Anouk_PrepareDirectThem : public Emiel_Prepare {
    public:
        Anouk_PrepareDirectThem(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        void Initialize();
        Status Update();

        ros::NodeHandle nh;

    private:
        std::vector<boost::uuids::uuid> tokens;

        std::vector<int> GetRobotsToDefend();
        std::vector<int> robotsToDefend;

    };

}