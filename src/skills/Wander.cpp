#include "roboteam_tactics/skills/Wander.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#define RTT_CURRENT_DEBUG_TAG Wander

namespace rtt {

RTT_REGISTER_SKILL(Wander);
    
Box Wander::quadTopLeft;
Box Wander::quadBottomLeft;
Box Wander::quadTopRight;
Box Wander::quadBottomRight;    
Box Wander::leftGoal;
Box Wander::rightGoal;
bool Wander::defaultsInitialized = false;
    
WanderArea areaForName(const std::string& name) {
    if (name == "NEAR_BALL") return WanderArea::NEAR_BALL;
    if (name == "NEAR_OUR_GOAL") return WanderArea::NEAR_OUR_GOAL;
    if (name == "NEAR_THEIR_GOAL") return WanderArea::NEAR_THEIR_GOAL;
    if (name == "QUADRANT") return WanderArea::QUADRANT;
    if (name == "BOX") return WanderArea::BOX;
    return WanderArea::NEAR_OUR_GOAL;
}    
    
void Wander::lazyInitDefaults() {
    if (defaultsInitialized || !LastWorld::have_received_first_geom()) return;
    defaultsInitialized = true;
    auto geom = LastWorld::get_field();
    double length = geom.field_length / 2;
    double width = geom.field_width / 2;
    
    quadTopLeft     = { { -length, 0      }, length, width };
    quadBottomLeft  = { { -length, -width }, length, width };
    quadTopRight    = { { 0      , 0      }, length, width };
    quadBottomRight = { { 0      , -width }, length, width };
    
    leftGoal        = { { -length      , -.75  }, 1.5, 1.5 }; 
    rightGoal       = { {  length - 1.5, -.75  }, 1.5, 1.5 }; 
}    
    
Wander::Wander(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {
    assert_valid<Wander>(name);
    currentDestinationValid = false;
    lazyInitDefaults();
}    

void Wander::configure() {
    std::string type = GetString("type");
    auto world = LastWorld::get();
    Vector2 ballPos(world.ball.pos);
    switch (areaForName(type)) {
        case WanderArea::NEAR_BALL: {
            wanderArea = { { ballPos.x - NEAR_BALL_DIST, ballPos.y - NEAR_BALL_DIST },
                           NEAR_BALL_DIST * 2, NEAR_BALL_DIST * 2 };
            break;
        }
        case WanderArea::NEAR_OUR_GOAL: {
            wanderArea = we_are_left() ? leftGoal : rightGoal;
            break;
        }
        case WanderArea::NEAR_THEIR_GOAL: {
            wanderArea = we_are_left() ? rightGoal : leftGoal;
            break;
        }
        case WanderArea::QUADRANT: {
            if (!HasInt("quadrant")) {
                throw std::invalid_argument("No quadrant specified, but WanderArea::QUADRANT selected");
            }
            static Box quads[4] = { quadTopLeft, quadTopRight, quadBottomLeft, quadBottomRight };
            wanderArea = quads[GetInt("quadrant")];
            break;
        }
        case WanderArea::BOX: {
            if (  !HasDouble("boxCenterX")
               || !HasDouble("boxCenterY")
               || !HasDouble("boxLength")) {
               throw std::invalid_argument("boxCenterX, boxCenterY and boxLength were not all specified");
            }
            double cx = GetDouble("boxCenterX");
            double cy = GetDouble("boxCenterY");
            double length = GetDouble("boxLength");
            double width = GetDouble("boxWidth", length);
            wanderArea = { { cx - width / 2, cy + length / 2 }, length, width };
            break;
        }
        default: throw std::logic_error("Incomplete switch in Wander::configure");
    }
}

void Wander::pickDestination() {
    double x = get_rand_real((float) wanderArea.length);
    double y = get_rand_real((float) wanderArea.width);
    std::string gtpName = "Wander_internal_GoToPos";
    
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
    ScopedBB(*bb, gtpName)
        .setBool("isKeeper", false)
        .setDouble("angleGoal", 0) // TODO?
        .setDouble("xGoal", wanderArea.bottomRight.x + x)
        .setDouble("yGoal", wanderArea.bottomRight.y + y)
        .setBool("dribbler", false)
        .setBool("avoidRobots", true);
    currentDestination = std::make_shared<Approach>(gtpName, bb);
    currentDestinationValid = true;
}
    
bt::Node::Status Wander::Update() {
    configure();
    if (!currentDestinationValid) pickDestination();
    
    bt::Node::Status gtpStatus = currentDestination->Update();
    
    if (gtpStatus != bt::Node::Status::Running) {
        if (gtpStatus == bt::Node::Status::Invalid) {
            return gtpStatus;
        }
        currentDestination.reset();
        pickDestination();
    }
    
    return bt::Node::Status::Running;
}    

}