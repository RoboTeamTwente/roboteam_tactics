#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Section.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include <boost/optional.hpp>

namespace rtt {

/**
 * \class BlockGoal
 * \brief See YAML
 */
/**
 * Descr: TODO
 * 
 * Params:
 *  - ROBOT_ID:
 *     Descr:     Id of the robot
 *     Type:      Int
 *
 *  - rotationFromOptimal:
 *     Descr:     TODO
 *     Type:      Double
 */
 
class BlockGoal : public Skill {
    public:
    BlockGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
    private:
    
    static boost::optional<roboteam_msgs::GeometryFieldSize> geom;
    static Arc topArc, bottomArc;
    static Section straightSection;
    bool leftSide;
    Vector2 center;
    Vector2 calcBlockPoint(const Vector2& target) const;
};
   
}
