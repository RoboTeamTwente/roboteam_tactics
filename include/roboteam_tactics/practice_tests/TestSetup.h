#pragma once

#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"
#include "PracticeTest.h"
#include <map>
#include <vector>

namespace rtt {
    
namespace practice {
    
typedef struct {
    std::map<TeamRobot, roboteam_utils::Position> bots;
    roboteam_utils::Vector2 ball_pos;
    roboteam_utils::Vector2 ball_vel;
} TestSetup;

class SetupBuilder {
    public:
    static SetupBuilder* builder();
    static Config reset();
    
    SetupBuilder* with_bot(const TeamRobot& bot, const roboteam_utils::Position& pos, const roboteam_utils::Position& vel = {0, 0, 0});
    SetupBuilder* combine(const Config& other);
    SetupBuilder* ignore_bot(const TeamRobot& bot);
    SetupBuilder* ignore_bots(const std::vector<TeamRobot>& bots);
    SetupBuilder* with_ball(const roboteam_utils::Vector2& pos);
    SetupBuilder* with_ball_vel(const roboteam_utils::Vector2& vel);
    SetupBuilder* stationary_ball();
    Config build() const;
    
    private:
    Config setup;
};
    
void send_setup_to_grsim(const Config& setup);    
    
}

}