#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

class DemoTactic : public Tactic {
    public:
    DemoTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    private:
    std::vector<boost::uuids::uuid> tokens;

    time_point start;
} ;

}
