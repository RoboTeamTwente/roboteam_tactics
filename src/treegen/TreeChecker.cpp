#include <boost/optional.hpp>
#include <tuple>
#include <string>
#include <set>

#include "roboteam_tactics/treegen/json.hpp"

// checkTree :: Tree -> Bool
// checkTree tree = checkTree' tree (root tree)
// 
// checkTree' :: Tree -> (Maybe Node) -> Bool
// checkTree' tree current = case current of
//     Sequence || Priority, geen children -> true
//     Sequence || Priority, 1 child -> checkTree child
//     Sequence || Priority, 2 child -> (allNonMem || onlyLastNonMem) && childrenAreOK
//         where
//             allNonMem = foldl1 (&&) $ map isNonMem children
//             onlyLastNonMem = foldl1 (&&) $ map isNonmem $ init children
//             childrenAreOK = foldl1 (&&) $ map checkTree children
//     Any other composite -> foldl1 (&&) $ map checkTree children

using nlohmann::json;
namespace b = boost;
using namespace std::string_literals;

// What we need:
// - Function that gives back type (condition, action, composite, decorator
//     - Condition: custom node
//     - Action: custom node
//     - composite: children array entry
//     - decorator: child entry

enum class NodeType {
    Condition,
    Action,
    Composite,
    Decorator
} ;

class NodeTyper {
public:
    NodeTyper(json const & treeData) {
        // Create conditions/actions database
        auto customNodesIt = treeData.find("customNodes");
        if (customNodesIt != treeData.end()) {
            auto customNodes = *customNodesIt;
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
                    actions.insert(name);
                } else {
                    warnings.push_back("Custom node with name " + name + " has unknown category \"" + category + "\".");
                }
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
    } else {
        throw std::runtime_error("Node ID " + currentID + " has no children attribute.");
    }
}

bool isNonMem(NodeTyper const & typer, json const & nodes, std::string const & nodeID) {
    // isNonMem holds when:
    // - it's a condition
    // - when it's a non-mem composite and it's children are all nonmem
    // - or an inverter decorator
    auto nodeIt = nodes.find(nodeID);

    if (nodeIt == nodes.end()) {
        throw std::runtime_error("Node ID " + nodeID + " does not exist in this tree.");
    }

    auto const node = *nodeIt;

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
                                okay = okay && isNonMem(typer, node, child);
                                if (!okay) break;
                            }

                            return okay;
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
    } else {
        throw std::runtime_error("Node ID " + nodeID + " has no derivable type.");
    }
}

bool checkTree(NodeTyper const & typer, json const & nodes, std::string const & currentID) {
    auto currentIt = nodes.find(currentID);
    if (currentIt == nodes.end()) {
        throw std::runtime_error("Node ID " + currentID + " not found in node list.");
    }

    auto const & currentNode = *currentIt;

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
    } else {
        throw std::runtime_error("Node ID " + currentID + " has no derivable type."); 
    }
}

/**
 * Checks if the Mem-variant holds for the given json tree
 */
std::tuple<b::optional<bool>, b::optional<std::string>> checkTree(json const & treeData) {
    auto nodesIt = treeData.find("nodes");
    auto rootIt = treeData.find("root");

    if (nodesIt == treeData.end() || rootIt == treeData.end()) {
        return std::make_tuple<b::optional<bool>, b::optional<std::string>>(b::none, "No root or nodes dictionary entry found"s);
    }

    NodeTyper nt(treeData);

    // TODO: Read out nt warnings

    try {
        return std::make_tuple(checkTree(nt, *nodesIt, rootIt->get<std::string>()), b::none);
    } catch (std::runtime_error const & e) {
        return std::make_tuple<b::optional<bool>, b::optional<std::string>>(b::none, std::string(e.what()));
    }
}
