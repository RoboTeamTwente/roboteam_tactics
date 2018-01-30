#pragma once

#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"

namespace rtt {

/**
 * \class MirrorOpponent
 * \brief See YAML
 */
/*
 * Descr: Mirror an opponent's position along the center line or some other x-coordinate.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The robot which is to shoot
 *   - targetId:
 *       Type: Int
 *       Descr: The opposing robot to mirror
 *   - mirrorX:
 *       Type: Double
 *       Descr: The x-coordinate to mirror the opponent on. By default, this is the center line.
 *       Default: 0
 *   - alongMirror:
 *       Type: Bool
 *       Descr: |
 *               If true, the robot will stay near the mirror line, otherwise it will maintain
 *               the same distance the opponent does.
 *       Default: true
 */
class MirrorOpponent : public Skill {
public:
	MirrorOpponent(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;
private:
	std::unique_ptr<GoToPos> gtp;
};

}
