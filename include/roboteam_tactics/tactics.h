// Tactics node
class WorldContainer {
	static roboteam_msgs::World getWorld();
	void setWorld(World world);
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
	Role(Aggregator& aggregator, bt::Blackboard::Ptr& blackboard);
} ;

class Skill : public bt::Leaf {
	Skill(Aggregator& aggregator, bt::Blackboard::Ptr& blackboard);
} ;

// 
