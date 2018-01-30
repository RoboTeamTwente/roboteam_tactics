#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/BiMap.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_msgs/World.h"
#include "Approach.h"

namespace rtt {

/**
 * \class Wander
 * \brief See YAML
 */
/*
 * Descr: Has a robot randomly move around within a predefined area.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - type:
 *       Type: String
 *       Can be:
 *         - NEAR_BALL: Wander near the ball
 *         - NEAR_OUR_GOAL: Wander near our goal
 *         - NEAR_THEIR_GOAL: Wander near the opponents' goal
 *         - QUADRANT: Wander in a specified (see below) quadrant of the field
 *         - BOX: Wander in a specified (see below) rectangular area
 *       Descr: Specifies the wander mode
 *   - quadrant:
 *       Type: Int
 *       Can be:
 *         - 0: Top-left
 *         - 1: Top-right
 *         - 2: Bottom-left
 *         - 3: Bottom-right
 *       Used when: type == QUADRANT
 *       Descr: Specifies what quadrant to wander in
 *   - boxCenterX:
 *        Type: Double
 *        Used when: type == BOX
 *        Descr: The x-coordinate of the center of the area to wander in
 *   - boxCenterY:
 *        Type: Double
 *        Used when: type == BOX
 *        Descr: The y-coordinate of the center of the area to wander in
 *   - boxLength:
 *        Type: Double
 *        Used when: type == BOX
 *        Descr: The length of the area to wander in
 *   - boxWidth:
 *        Type: Double
 *        Used when: type == BOX
 *        Default: boxLength
 *        Descr: The width of the area to wander in
 */



/**
 * \enum WanderArea
 * \brief The different types of wandering robots can do.
 */
enum class WanderArea {
    NEAR_BALL,        //< Wander near the ball
    NEAR_OUR_GOAL,    //< Wander near our goal
    NEAR_THEIR_GOAL,  //< Wander near the opponents' goal
    QUADRANT,         //< Wander in a quadrant (numbered 0-3, left to right, top to bottom) of the field
    BOX               //< Wander in a rectangular area specified in the blackboard
}; 

/**
 * \brief Gets the WanderArea corresponding to a string
 */ 
WanderArea areaForName(const std::string& name);

/**
 * \struct Box
 * \brief Defines a rectangle by giving its bottom-right corner, length and width.
 */
struct Box {
    Vector2 bottomRight;
    double length;
    double width;
};

class Wander : public Skill {
public:
    static constexpr double NEAR_BALL_DIST = .8;
    static constexpr double NEAR_GOAL_DIST = 1;
    
    Wander(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
private:
    static Box quadTopLeft;
    static Box quadBottomLeft;
    static Box quadTopRight;
    static Box quadBottomRight;
    static Box leftGoal;
    static Box rightGoal;
    static bool defaultsInitialized;
    
    static void lazyInitDefaults();
    
    Box wanderArea;
    void configure();
    
    std::shared_ptr<Approach> currentDestination;
    bool currentDestinationValid;
    
    void pickDestination();
};
  
}
