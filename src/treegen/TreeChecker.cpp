#include <boost/optional.hpp>
#include <tuple>
#include <string>
#include <set>

#include "roboteam_tactics/treegen/json.hpp"
#include "roboteam_tactics/treegen/TreeChecker.h"

/**
 * This file contains all the logic to check trees for the mem principle.
 *
 * ***********************
 * ** The mem principle **
 * ***********************
 *
 * When you have a sequence (not a mem-sequence!) which has two skills/actions
 * (nodes that can return running), what happens when the first node returns success,
 * the second node returns running? The node will be paused, and the following frame
 * updated again. Now this is where the problem happens: instead of running the 
 * action which returned running the previous frame, it runs the first node (the
 * one that returned success last frame)! Normally this is not a problem, but
 * once the actions start allocating resources (e.g. Plays do this when they allocate
 * players or the keeper via RobotDealer!) this might bite you. Therefore, to adhere 
 * to the mem principle, every tree must ensure that:
 *
 * Every sequence or priority has either:
 * - only conditions
 * - or only conditions, with exception of the last child, which is allowed to
 *   return running.
 *
 * That way the scenario described above can never occur.
 *
 * ***************
 * ** Structure **
 * ***************
 *
 * The idea is that checkTree just returns a boolean whether or not the tree obeys
 * the mem-principle. However, since we're dealing with JSON here, there can be errors
 * (members missing, null members, etc.). So anywhere where a weird error occurs,
 * be it a JSON error or something else, a runtime error can be thrown with a textual
 * description of the problem. This exception is then caught in the toplevel checkTree
 * function, which can return a sum-like either type with the results.
 *
 * Why was this way picked and not a proper either type (without the monad, because
 * c++ does not support that)? Then the results of the checktree
 * and isNonMem function would have to be unpacked everywhere, which is noisy and doesn't
 * add any real value to the code. In a constrained environment like this an exception
 * is a nice analogy to the Either monad.
 *
 */

namespace {

using nlohmann::json;
namespace b = boost;
using namespace std::string_literals;

/**
 * Enum used to denote a node type in a Behavior3 behavior tree.
 */
enum class NodeType {
    Condition,
    Action,
    Composite,
    Decorator
} ;

/**
 * Class that, given a list of custom nodes used in a tree, can tell you based
 * on the node data, what type the node is.
 *
 * Criteria:
 * - If it has a children member, it's a composite.
 * - If it has a child member, it's a decorator.
 * - If its name occurs in the actions category, it's an action
 * - If its name occurs in the condition category, it's a condition.
 */
class NodeTyper {
public:
    NodeTyper(json const & customNodes) :
            actions {
                "Failer",
                "Succeeder",
                "Error",
                "Runner"
            },
            conditions {
                // No default conditions
            } {
        for (auto const & customNode : customNodes) {
            auto nameIt = customNode.find("name");
            if (nameIt == customNode.end()) {
                warnings.push_back("A custom node has no name attribute.");
                continue;
            }
            auto const name = nameIt->get<std::string>();

            auto categoryIt = customNode.find("category");
            if (categoryIt == customNode.end()) {
                warnings.push_back("Custom node with name \"" + name + "\" has no category.");
                continue;
            }
            auto const category = categoryIt->get<std::string>();

            if (category == "action") {
                actions.insert(name);
            } else if (category == "condition") {
                conditions.insert(name);
            } else {
                warnings.push_back("Custom node with name " + name + " has unknown category \"" + category + "\".");
            }
        }
    }

    b::optional<NodeType> getType(json const & node) const {
        auto childrenIt = node.find("children");
        if (childrenIt != node.end()) {
            return NodeType::Composite;
        }

        auto childIt = node.find("child");
        if (childIt != node.end()) {
            return NodeType::Decorator;
        }

        auto nameIt = node.find("name");
        if (nameIt != node.end()) {
            auto const name = nameIt->get<std::string>();
            if (actions.find(name) != actions.end()) {
                return NodeType::Action;
            } else if (conditions.find(name) != conditions.end()) {
                return NodeType::Condition;
            }
        }

        return b::none;
    }

    std::vector<std::string> const & getWarnings() const {
        return warnings;
    }

private:
    std::set<std::string> actions;
    std::set<std::string> conditions;
    std::vector<std::string> warnings;
} ;

/**
 * Returns the name of the node if it's there.
 */
b::optional<std::string> getNodeName(json const & node) {
    auto nameIt = node.find("name");
    if (nameIt != node.end()) {
        return nameIt->get<std::string>();
    }

    return b::none;
}

/**
 * Returns a list of children node id's if it's there.
 */
b::optional<std::vector<std::string>> getChildren(json const & node) {
    auto childrenIt = node.find("children");
    if (childrenIt != node.end()) {
        auto const & children = *childrenIt;

        std::vector<std::string> childrenIDs;

        for (auto const & child : children) {
            childrenIDs.push_back(child.get<std::string>());
        }

        return childrenIDs;
    }

    return b::none;
}

/**
 * Returns the child id if it's there.
 */
b::optional<std::string> getChild(json const & node) {
    auto nameIt = node.find("child");
    if (nameIt != node.end()) {
        return nameIt->get<std::string>();
    }

    return b::none;
}

bool checkTree(NodeTyper const & typer, json const & nodes, std::string const & currentID);

/**
 * Returns true if checkTree returns true for all the children if the given node.
 */
bool childrenAreOkay(NodeTyper const & typer, json const & nodes, std::string const & currentID, json const & currentNode) {
    if (auto const childrenOpt = getChildren(currentNode)) {
        bool okay = true;

        for (auto const & child : *childrenOpt) {
            okay = okay && checkTree(typer, nodes, child);
            if (!okay) break;
        }

        return okay;
    } else {
        throw std::runtime_error("Node ID " + currentID + " has no children attribute.");
    }

}

/**
 * Checks whether or not a node is "non-mem", i.e. whether or not
 * it can return Running. A node is non-mem when:
 * - It's a condition
 * - When it's a non-mem composite (Sequence or Priority) and it's children are all non-mem
 * - An inverter decorator whose child is also non-mem.
 */
bool isNonMem(NodeTyper const & typer, json const & nodes, std::string const & nodeID) {
    auto nodeIt = nodes.find(nodeID);

    if (nodeIt == nodes.end()) {
        throw std::runtime_error("Node ID \"" + nodeID + "\" does not exist in this tree.");
    }

    auto const node = *nodeIt;

    b::optional<std::string> name;
    if (node.find("title") != node.end()) {
        name = node["title"].get<std::string>();
    }

    if (auto nodeTypeOpt = typer.getType(node)) {
        auto const nodeType = *nodeTypeOpt;

        switch (nodeType) {
            case NodeType::Decorator:
                if (auto nameOpt = getNodeName(node)) {
                    auto const name = *nameOpt;
                    if (name == "Inverter") {
                        if (auto const childOpt = getChild(node)) {
                            return isNonMem(typer, nodes, *childOpt);
                        } else {
                            throw std::runtime_error("Node ID \"" + nodeID + "\" with name \"" + name + "\" has no child.");
                        }
                    } else {
                        return false;
                    }
                } else {
                    throw std::runtime_error("Node ID \"" + nodeID + "\" has no node name");
                }

                break;
            case NodeType::Condition:
                return true;
            case NodeType::Composite:
                if (auto nameOpt = getNodeName(node)) {
                    auto const name = *nameOpt;
                    if (name == "Sequence" || name == "Priority") {
                        if (auto childrenOpt = getChildren(node)) {
                            auto const children = *childrenOpt;

                            bool okay = true;
                            for (auto const & child : children) {
                                okay = okay && isNonMem(typer, nodes, child);
                                if (!okay) break;
                            }

                            return okay;
                        } else {
                            throw std::runtime_error("Node ID \"" + nodeID + "\" has no children even though it is a sequence or priority. This is a broken tree.");
                        }
                    } else {
                        return false;
                    }
                } else {
                    throw std::runtime_error("Node ID \"" + nodeID + "\" has no node name");
                }

                break;
            case NodeType::Action:
                return false;
        }

        throw std::runtime_error("Node ID \"" + nodeID + "\" did not match a type in isNonMem. This is a bug!");
    } else {
        if (name) {
            throw std::runtime_error("Node \"" + *name + "\" has no derivable type.");
        } else {
            throw std::runtime_error("Node ID " + nodeID + " has no derivable type.");
        }
    }
}

/**
 * Checks if the subtree starting from currentID obeys the mem-principle.
 */
bool checkTree(NodeTyper const & typer, json const & nodes, std::string const & currentID) {
    auto currentIt = nodes.find(currentID);
    if (currentIt == nodes.end()) {
        throw std::runtime_error("Node ID " + currentID + " not found in node list.");
    }

    auto const & currentNode = *currentIt;

    b::optional<std::string> name;
    if (currentNode.find("title") != currentNode.end()) {
        name = currentNode["title"].get<std::string>();
    }

    if (auto const nodeTypeOpt = typer.getType(currentNode)) {
        auto const nodeType = *nodeTypeOpt;
        switch (nodeType) {
            case NodeType::Decorator:
                return checkTree(typer, nodes, currentNode["child"].get<std::string>());
            case NodeType::Action:
                return true;
            case NodeType::Condition:
                return true;
            case NodeType::Composite:
                if (auto nameOpt = getNodeName(currentNode)) {
                    auto name = *nameOpt;
                    if (name == "Sequence" || name == "Priority") {
                        // Interesting case
                        if (auto childrenOpt = getChildren(currentNode)) {
                            auto const children = *childrenOpt;

                            if (children.size() == 0) {
                                return true;
                            } else if (children.size() == 1) {
                                return checkTree(typer, nodes, children[0]);
                            } else {
                                // First check allNonMem
                                // Init == all but the last
                                bool initNonMem = true;

                                for (size_t i = 0; i < (children.size() - 1); ++i) {
                                    auto const & child = children.at(i);
                                    initNonMem = initNonMem && isNonMem(typer, nodes, child);
                                }

                                return initNonMem && childrenAreOkay(typer, nodes, currentID, currentNode);
                            }
                        } else {
                            throw std::runtime_error("Node ID " + currentID + " has no children.");
                        }
                    } else {
                        return childrenAreOkay(typer, nodes, currentID, currentNode);
                    }
                } else {
                    throw std::runtime_error("Node ID " + currentID + " has no name attribute.");
                }
                break;
        }

        throw std::runtime_error("Node ID \"" + currentID + "\" did not match a node type in checkTree. This is a bug!");
    } else {
        if (name) {
            throw std::runtime_error("Node \"" + *name + "\" has no derivable type."); 
        } else {
            throw std::runtime_error("Node ID \"" + currentID + "\" has no derivable type."); 
        }
    }
}

} // anonymous namespace

namespace rtt {

/**
 * Checks if the mem-principle holds for the given json tree. The customnodes list
 * can either be found in the tree or in the project, depending on if just a tree
 * is processed or a whole project.
 */
std::tuple<CheckResult, b::optional<std::string>> checkTree(json const & treeData, json const & customNodes) {
    auto nodesIt = treeData.find("nodes");
    auto rootIt = treeData.find("root");

    if (nodesIt == treeData.end() || rootIt == treeData.end()) {
        return std::make_tuple<CheckResult, b::optional<std::string>>(CheckResult::Error, "No root or nodes dictionary entry found"s);
    }

    NodeTyper nt(customNodes);

    // TODO: Read out nt warnings
    
    if (rootIt->is_null()) {
        return std::make_tuple(CheckResult::Good, b::none);
    }

    if (!rootIt->is_string()) {
        return std::make_tuple<CheckResult, b::optional<std::string>>(CheckResult::Error, "Root node ID is not a string!"s);
    }

    try {
        bool res = checkTree(nt, *nodesIt, rootIt->get<std::string>());

        return std::make_tuple(res ? CheckResult::Good : CheckResult::Bad, b::none);
    } catch (std::runtime_error const & e) {
        return std::make_tuple<CheckResult, b::optional<std::string>>(CheckResult::Error, std::string(e.what()));
    }
}

} // rtt
