#include "roboteam_tactics/utils/LeafRegister.h"

rtt::factories::TreeRegisterer::TreeRegisterer(const std::string &name, TreeConstructor tc) {
    auto treeFactory = [=](std::string name, bt::Blackboard::Ptr bb) {
        // Name is not used
        return std::make_shared<bt::BehaviorTree>(tc(bb.get()));
    };

    std::cout << "Registering tree " << name << "\n";

    getRepo<Factory<bt::BehaviorTree>>()[name] = treeFactory;
    std::cout << "Repo size: " << getRepo<Factory<bt::BehaviorTree>>().size() << "\n";
    std::cout << "Repo location (constructor): " << &getRepo<Factory<bt::BehaviorTree>>() << "\n";
}

// // Repo<TreeFactory>& getFactoryRepo() {
    // // static Repo<TreeFactory> repo;

    // // return repo;
// // }

// // Repo<Factory<Condition>>& getConditionFactoryMap() {
    // // static Repo<Factory<Condition>> repo;


    // // return repo;
// // }

// // Repo<Factory<Skill>>& getSkillFactoryMap() {
    // // static Repo<Factory<Skill>> repo;

    // // return repo;
// // }

// // Repo<Factory<Tactic>>& getTacticFactoryMap() {
    // // static Repo<Factory<Tactic>> repo;

    // // return repo;
// // }

// } // factories

// } // rtt
