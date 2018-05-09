#pragma once

#include <chrono>

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class Anouk_BallPlacementThemPlay : public Tactic {
public:
    Anouk_BallPlacementThemPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

	void Initialize() override;
    Status Update() override;
    
    std::string node_name() override { return "Anouk_BallPlacementThemPlay"; }

private:
    boost::uuids::uuid token;

	Vector2 lastBallPlacementPosition;
} ;

} // rtt


