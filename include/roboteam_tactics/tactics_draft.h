// Tactics node
class LastWorld {
	static roboteam_msgs::World get();
	void set(World world);
}

// WorldState message
// subscriber die die update ^^

// Eventueel als blackboards te crowded worden, een blackboard stack
// Nieuw blackboard pushen als een nieuwe tactic wordt uitgevoerd
// Tactics heeft maar 1 blackboard, of iets vergelijkbaars per rol.

class Aggregator {
	std::map<int, SteeringActionGoal> instructions;
	std::map<int, SteeringActionResult> feedbacks;
} ;

class Strategy : public bt::BehaviorTree {
	
} ;

class Tactic : public bt::Leaf {	
	std::vector<Role> roles;
} ;

class Role : public bt::BehaviorTree {
	Role(Aggregator& aggregator, bt::Blackboard::Ptr& blackboard, int robotId);
} ;

class Skill : public bt::Leaf {
	Skill(Aggregator& aggregator, bt::Blackboard::Ptr& blackboard);
} ;

// Example implementation of skill
// (I know this is better implemented as a sequence of two or tree fine grained skills, but w/e)

class AcquireBallAndDribbleToPoint : public Skill {
	AcquireBallAndDribbleToPoint(Aggregator& aggregator, bt::Blackboard::Ptr& blackboard, int robotId, double x, double y) :
			Skill(aggregator, blackboard) {
		robotId = robotId;
		tx = x;
		ty = y;
	}
	
	Status Update() override {
		roboteam_msgs::World world = LastWorld::get();

		Pos robotPos = LastWorld::getRobotPos(robotId);

		if (robotPos == Vector2d(tx, ty) && LastWorld::hasBall(robotId)) {
			return bt::Node::Status::Success;
		}

		SteeringActionGoal sag;

		if (robotHasBall(id)) {
			// Obviously wrong, but at least should set the right direction
			sag.vx = tx;
			sag.vy = ty;
		} else {
			Pos pos = getBallPosition(LastWorld::getBallPosition())
			sag.vx = pos.x;
			sag.vy = pos.y;
		}

		aggregator.submit(robotId, robotCommand);

		return bt::Node::Status::Running;
	}
	
}
