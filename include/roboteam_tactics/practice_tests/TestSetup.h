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

/**
 * \struct TestSetup
 * \brief A static setup used to prepare for a test
 */    
typedef struct {
    std::map<TeamRobot, Position> bots;
    Vector2 ball_pos;
    Vector2 ball_vel;
} TestSetup;

/**
 * \class SetupBuilder
 * \brief Fluid builder for a test Config
 */
class SetupBuilder {
    public:
    
    /**
     * \return A new builder 
     */
    static SetupBuilder* builder();
    
    /**
     * \return Get a config which will set all robots to the side line.
     */
    const static Config reset();
    
    /**
     * \brief Add a robot to the Config
     * \param bot The robot to add
     * \param pos The position this robot should be at
     * \param vel The velocity this robot should have
     * \return This SetupBuilder
     */
    SetupBuilder* with_bot(const TeamRobot& bot, const Position& pos, const Position& vel = {0, 0, 0});
    
    /**
     * \brief Combine the Config under construction with another
     * \param other The other Config to incorporate
     * \return This SetupBuilder
     */
    SetupBuilder* combine(const Config& other);
    
    /**
     * \brief Relegate a robot to the side.
     * \param bot The robot to ignore
     * \return This SetupBuilder
     */
    SetupBuilder* ignore_bot(const TeamRobot& bot);
    
    /**
     * \brief Relegates multiple robots to the side
     * \param bots The robots to ignore
     * \return This SetupBuilder
     */
    SetupBuilder* ignore_bots(const std::vector<TeamRobot>& bots);
    
    /**
     * \brief Sets the ball position
     * \param pos The position to set
     * \return  This SetupBuilder
     */
    SetupBuilder* with_ball(const Vector2& pos);
    
    /**
     * \brief Sets the ball velocity 
     * \param vel The velocity to set
     * \return This SetupBuilder
     */
    SetupBuilder* with_ball_vel(const Vector2& vel);
    
    /**
     * \brief Sets the ball to be stationary
     * \return This SetupBuilder
     */
    SetupBuilder* stationary_ball();
    
    /**
     * \brief Finish building and return the Config
     */
    Config build() const;
    
    private:
    Config setup;
};
    
/**
 * \brief Apply a config by sending it to grsim.
 */
void send_setup_to_grsim(const Config& setup);    
    
}

}