#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Block.h"
#include "roboteam_tactics/skills/GetBall.h"

namespace rtt {

/*
 * Descr: >
 *   Depending on whether or not the target opponent has the ball, this skill will either try to
 *   prevent the opponent from passing or receiving it.
 * 
 * Global params: 
 *   ROBOT_ID:
 *     Type: int
 *     Descr: ID of the harasser.
 *   TGT_ID:
 *     Type: int
 *     Descr: ID of the target.
 *   distance:
 *     Type: double
 *     Descr: Distance the harasser will keep to the target.
 *     Default: 0.4
 */ 
class Harass : public Skill {
    public:
    Harass(std::string name = "", bt::Blackboard::Ptr bb = nullptr);
    Status Update() override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["TGT_ID"] = BBArgumentType::Int;
        return params;
    }
    
    private:
    std::unique_ptr<Block> block_get, block_kick;
    std::unique_ptr<GetBall> get_ball;
    int target;

};
    
}