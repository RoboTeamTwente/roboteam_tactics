#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"

namespace rtt {

class ThrowinPlay : public Tactic {
    public:
    ThrowinPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "ThrowinPlay"; }
    private:
    std::vector<boost::uuids::uuid> tokens;
} ;

}
