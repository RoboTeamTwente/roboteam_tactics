#include "roboteam_tactics/rules/TimeoutRule.h"

namespace rtt {
    
bool TimeoutRule::canMove(const TeamRobot&) const { return false; }
bool TimeoutRule::canMove(const TeamRobot&, const Vector2&) const { return false; }

bool TimeoutRule::canDribble(const TeamRobot&) const { return false; }
bool TimeoutRule::canDribble(const TeamRobot&, const Vector2&) const { return false; }
   
bool TimeoutRule::canKick(const TeamRobot&) const { return false; }
    
bool TimeoutRule::canChip(const TeamRobot&) const { return false; }
    
double TimeoutRule::maxSpeed(const TeamRobot&) const { return 0.0; }
    
const RefRule* TimeoutRule::Us::SINGLETON = new Us();
const RefRule* TimeoutRule::Us::get() { return SINGLETON; }

const RefRule* TimeoutRule::Them::SINGLETON = new Them();
const RefRule* TimeoutRule::Them::get() { return SINGLETON; }
    
}