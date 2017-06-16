#include <boost/optional.hpp>
#include <tuple>
#include <string>
#include <set>

#include "roboteam_tactics/treegen/json.hpp"
#include "roboteam_tactics/treegen/TreeChecker.h"

namespace {

using nlohmann::json;
namespace b = boost;
using namespace std::string_literals;

enum class NodeType {
    Condition,
    Action,
    Composite,
    Decorator
} ;

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

b::optional<std::string> getNodeName(json const & node) {
    auto nameIt = node.find("name");
    if (nameIt != node.end()) {
        return nameIt->get<std::string>();
    }

    return b::none;
}

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

b::optional<std::string> getChild(json const & node) {
    auto nameIt = node.find("child");
    if (nameIt != node.end()) {
        return nameIt->get<std::string>();
    }

    return b::none;
}

bool checkTree(NodeTyper const & typer, json const & nodes, std::string const & currentID);

bool childrenAreOkay(NodeTyper const & typer, json const & nodes, std::string const & currentID, json const & currentNode) {
    if (auto const childrenOpt = getChildren(currentNode)) {
        bool okay = true;

        for (auto const & child : *childrenOpt) {
            okay = okay && checkTree(typer, nodes, child);
        }

        return okay;
    } else {
        throw std::runtime_error("Node ID " + currentID + " has no children attribute.");
    }

}

bool isNonMem(NodeTyper const & typer, json const & nodes, std::string const & nodeID) {
    // isNonMem holds when:
    // - it's a condition
    // - when it's a non-mem composite and it's children are all nonmem except the last one
    // - or an inverter decorator
    
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
                            for (size_t i = 0; i < children.size() - 1; ++i) {
                                auto const & child = children.at(i);
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
 * Checks if the Mem-variant holds for the given json tree
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
