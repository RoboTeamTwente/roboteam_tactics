#include <algorithm>
#include <numeric>
#include <stack>
#include <string>
#include <unordered_map>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
namespace bf = boost::filesystem;

#include "roboteam_tactics/treegen/BTBuilder.h"
#include "roboteam_tactics/treegen/json.hpp"

#define INDENT "    "
#define DINDENT "        "

using json = nlohmann::json;

namespace rtt {

BTBuilder::BTBuilder() {}
BTBuilder::~BTBuilder() {}

namespace {

std::string folderConcat(std::string left, std::string right) {
    if (left.empty()) {
        return right;
    } else if (right.empty()) {
        return left;
    } else {
        return left + "/" + right;
    }
}

std::vector<std::string> get_all_recursively(std::string category, std::string folders, bool recurse = true) {
    bf::path categoryPath(folderConcat("src/" + category, folders));

    std::vector<std::string> xs;
    try {
        if (exists(categoryPath)) {
            for (bf::directory_entry& de : bf::directory_iterator(categoryPath)) {
                auto p = de.path();

                if (bf::is_directory(p) && recurse) {
                    // Get the foldername
                    auto deeperFolder = p.stem().string();
                    // Create the new folder path to the deeper folder
                    auto newFolders = folderConcat(folders, deeperFolder);
                    // Find the deeper elements
                    auto deeperElements = get_all_recursively(category, newFolders, recurse);
                    // Insert the newly found elements in xs
                    xs.insert(xs.end(), deeperElements.begin(), deeperElements.end());
                } else if (p.extension().string() == ".cpp") {
                    xs.push_back(folderConcat(folders, de.path().stem().string()));
                }
            }
        } else {
            std::cerr << "Path " << categoryPath << " does not exist. Aborting.\n";
            return {};
        }

        return xs;
    } catch (const bf::filesystem_error& ex) {
        return {};
    }
}

std::vector<std::string> get_all(std::string category, bool recurse = true) {
    return get_all_recursively(category, "", recurse);
}

std::string current_tree;
bool encountered_error = false;

std::string const RED_BOLD_COLOR = "\e[1;31m";
std::string const YELLOW_BOLD_COLOR = "\e[1;33m";
std::string const END_COLOR = "\e[0m";

void cmakeErr(std::string msg) {
    std::cerr << "[----] " << RED_BOLD_COLOR << msg << END_COLOR << "\n";
}

void cmakeErrTree(std::string msg) {
    cmakeErr("Tree \"" + current_tree + "\": " + msg);
}

void cmakeWarn(std::string msg) {
    std::cerr << "[----] " << YELLOW_BOLD_COLOR << msg << END_COLOR << "\n";
}

void cmakeWarnTree(std::string msg) {
    cmakeWarn("Tree \"" + current_tree + "\": " + msg);
}

} // Anonymous namespace

boost::optional<std::string> BTBuilder::build(nlohmann::json json, std::string baseNamespace, std::vector<std::string> namespaces) {

    // namespace f = rtt::factories;

    // allskills_list = f::get_entry_names<Skill>();
    // allconditions_list = f::get_entry_names<Condition>();
    // alltactics_list = f::get_entry_names<Tactic>();

    allskills_list = get_all("skills");
    allconditions_list = get_all("conditions");
    alltactics_list = get_all("tactics");

    // Test code to print all the tactics
    if (false) {
        auto allTactics = get_all("tactics");

        std::cout << "-- Listing tactics\n";
        for (auto tactic : allTactics) {
            std::cout << "Tactic: " << tactic << "\n";
        }
    }

    // To turn every list into a set and clear the previous sets
    auto initializeSet = [](std::set<std::string> &theSet, std::vector<std::string> &theVector) {
        theSet.clear();
        theSet.insert(theVector.begin(), theVector.end());
    };

    initializeSet(allskills_set, allskills_list);
    initializeSet(allconditions_set, allconditions_list);
    initializeSet(alltactics_set, alltactics_list);

    std::stack<std::string> stack;
    try {
    stack.push(json["root"]);
    } catch (...) {
        std::cout << json;
        throw;
    }
    std::set<std::string> usedSkills, usedConditions, usedTactics, usedTitles;

    // auto& skillRepo = getRepo<Factory<Skill>>();
    // auto& conditionRepo = getRepo<Factory<Condition>>();
    // auto& tacticRepo = getRepo<Factory<Tactic>>();

    // Give all nodes who are not a skill nor condition
    // a unique name by appending a number at the end
    int ctr = 0;
    for (auto& element : json["nodes"]) {
        std::string title = element["title"];

        if (usedTitles.find(title) != usedTitles.end()) {
            cmakeErrTree("Nodename \"" + title + "\" appears more than once in the tree. Please use unique names, or the nodes might be unaddressable.");
        }

        if (allskills_set.find(element["name"].get<std::string>()) != allskills_set.end()) {
            usedSkills.insert(element["name"].get<std::string>());

            if (allskills_set.find(title) != allskills_set.end()) {
                cmakeErrTree("Skill \"" + title + "\" has the same name as the skill type. Please use a leaf name different from the skill name (e.g. " + title + "_A, " + title + "_1)");
            }
        }

        if (allconditions_set.find(element["name"].get<std::string>()) != allconditions_set.end()) {
            usedConditions.insert(element["name"].get<std::string>());

            if (allconditions_set.find(title) != allconditions_set.end()) {
                cmakeErrTree("Condition \"" + title + "\" has the same name as the condition type. Please use a leaf name different from the condition name (e.g. " + title + "_A, " + title + "_1)");
            }
        }

        if (alltactics_set.find(element["name"].get<std::string>()) != alltactics_set.end()) {
            usedTactics.insert(element["name"].get<std::string>());
        }

        if (allskills_set.find(element["name"]) == allskills_set.end()
                && allconditions_set.find(element["name"]) == allconditions_set.end()) {
            // Get title and append ctr
            title += "_" + std::to_string(ctr++);
        }

        // Replace spaces with _
        std::transform(title.begin(), title.end(), title.begin(), [](char ch) {
            return ch == ' ' ? '_' : ch;
        });
        // Put it back
        element["title"] = title;

        usedTitles.insert(title);
    }

    while (!stack.empty()) {
        std::string id = stack.top();
        stack.pop();
        nlohmann::json element = json["nodes"][id];
        nodes[id] = element;
        if (element.find("children") != element.end()) {
            for (auto child : element["children"]) {
                stack.push(child);
            }
        } else if (element.find("child") != element.end()) {
            stack.push(element["child"]);
        }
    }

    out << INDENT << "// Used skills: \n";
    for (const auto& skill : usedSkills) {
        out << INDENT << "#include \"roboteam_tactics/skills/" << skill << ".h\"\n";
    }

    out << INDENT << "// Used conditions: \n";
    for (const auto& condition : usedConditions) {
        out << INDENT << "#include \"roboteam_tactics/conditions/" << condition << ".h\"\n";
    }

    out << INDENT << "// Used tactics: \n";
    for (const auto& tactic : usedTactics) {
        out << INDENT << "#include \"roboteam_tactics/tactics/" << tactic << ".h\"\n";
    }
    out << "\n";

    // Create constructor function
    if (!baseNamespace.empty()) {
        out << INDENT << "namespace " << baseNamespace << " {\n\n";
    }

    std::string stringified_namespaces = "";

    if (namespaces.size() > 0) {
        out << INDENT;

        for (auto ns : namespaces) {
            out << "namespace " << ns << " { ";

            stringified_namespaces += ns + "/";
        }

        out << "\n\n";
    }

    current_tree = stringified_namespaces + json["title"].get<std::string>();

    out << INDENT << "bt::BehaviorTree make_"
        << json["title"].get<std::string>()
        << "(bt::Blackboard* blackboard) {"
        << std::endl;
    out << DINDENT << "bt::BehaviorTree tree;" << std::endl;
    out << DINDENT << "auto bb = tree.GetBlackboard();" << std::endl;
    out << DINDENT << "if(blackboard) merge_blackboards(bb, std::make_shared<bt::Blackboard>(*blackboard));" << std::endl;

    nlohmann::json root = nodes[json["root"]];
    defines(root);
    build_structure(root);

    out << DINDENT << "tree.SetRoot(" << root["title"].get<std::string>() << ");" << std::endl;
    out << DINDENT << "return tree;" << std::endl;
    out << INDENT << "}\n\n";

    if (namespaces.size() > 0) {
        out << INDENT;

        for (int i = namespaces.size() - 1; i > -1; --i) {
            out << "} /* " << namespaces.at(i) << " */ ";
        }

        out << "\n\n";
    }
    
    if (!baseNamespace.empty()) {
        out << INDENT << "} // " << baseNamespace << "\n\n";
    }

    std::string treeName = json["title"].get<std::string>();

    std::string namespacedFunction;
    {
        auto nss = namespaces;
        nss.push_back("make_" + treeName);

        namespacedFunction = 
        "&"
        + std::accumulate(
            nss.begin(),
            nss.end(),
            baseNamespace,
            [](std::string l, std::string r) {
                if (l.empty()) {
                    return r;
                }

                return l + "::" + r;
            }
            );
    }

    std::string creationString;
    {
        auto nss = namespaces;
        nss.push_back(treeName);

        creationString = std::accumulate(
            nss.begin(), 
            nss.end(), 
            std::string(""), 
            [](std::string l, std::string r) {
                if (l.empty()) {
                    return r;
                }

                return l + "/" + r;
            }
            );
    }

    out << INDENT << "namespace {\n\n";
    out << INDENT
        << "rtt::factories::TreeRegisterer rtt_"
        << treeName
        << "_registerer(\""
        << creationString
        << "\", "
        << namespacedFunction
        << ");\n\n";
    out << INDENT << "} // anonymous namespace\n";

    if (!encountered_error) {
        return out.str();
    } else {
        return boost::none;
    }
}

std::string BTBuilder::get_parallel_params_from_properties(json properties) {
    if (properties.find("minSuccess") != properties.end()
            && properties.find("minFail") != properties.end()) {
        int minSuccess = properties["minSuccess"];
        int minFail = properties["minFail"];
        return std::to_string(minSuccess)
            + ", "
            + std::to_string(minFail);
    } else if (properties.find("successOnAll") != properties.end()
            && properties.find("failOnAll") != properties.end()) {
        std::string successOnAll = properties["successOnAll"].get<std::string>();
        std::string failOnAll = properties["failOnAll"].get<std::string>();

        if ((successOnAll != "true" && successOnAll != "false") 
                && (failOnAll != "true" && failOnAll != "false")) {

            cmakeErrTree("failOnAll & successOnAll are not nice booleans!");

            encountered_error = true;
            return "true, false";
        }
        
        return successOnAll + ", " + failOnAll;
    } else {
        return "true, false";
    }
}

void BTBuilder::define_seq(std::string name, std::string nodeType, json properties = json::object()) {
    std::string type = "bt::Sequence";
    std::string params = "";

        if (nodeType == "MemSequence") {
        // It's a mem sequence
        type = "bt::MemSequence";
    } else if (nodeType == "ParallelSequence") {
        // Parallel sequence with repeat
        type = "bt::ParallelSequence";
        params = get_parallel_params_from_properties(properties);
    } else if (nodeType == "ParallelTactic") {
        // Parallel sequence without repeat
        type = "ParallelTactic";
        params = get_parallel_params_from_properties(properties);
    } else {
        // It's a regular sequence
        type = "bt::Sequence";
    }

    out << DINDENT << "auto " << name << " = std::make_shared<" << type << ">(" << params << ");\n";
}

void BTBuilder::define_sel(std::string name) {
    out << DINDENT << "auto " << name << " = std::make_shared<bt::Selector>();" << std::endl;
}

void BTBuilder::define_dec(std::string name, std::string type, json data) {
    std::string params = "";

    if (type == "Repeat") {
        type = "bt::Repeater";

        auto it = data.find("properties");
        if (it != data.end()) {
            auto properties = *it;
            auto limitIt = properties.find("limit");
            if (limitIt != properties.end()) {
                if (limitIt->is_number_integer()) {
                    params = std::to_string(limitIt->get<int>());
                } else {
                    if (!(limitIt->is_string() && limitIt->get<std::string>().empty())) {
                        cmakeErrTree("limit property of repeat node is not an integer.");
                        encountered_error = true;
                    }
                }
            }
        } 
    } else if (type == "RepeatUntilFailure") {
        type = "bt::UntilFail";
    } else if (type == "RepeatUntilSuccess") {
        type = "bt::UntilSuccess";
    } else if (type == "Inverter") {
        type = "bt::Inverter";
    } else {
        params = "bb";
    }

    out << DINDENT << "auto " << name << " = std::make_shared<" << type << ">(" << params << ");\n";
}

void BTBuilder::define_nod(std::string name, std::string type) {
    // If it's a skill we need to pass the node handle
    if (allskills_set.find(type) != allskills_set.end()) {
        out << DINDENT
            << "auto "
            << name
            << " = std::make_shared<"
            << type
            << ">(\""
            << name
            << "\", bb);"
            << std::endl;
    } else if (allconditions_set.find(type) != allconditions_set.end()
            || alltactics_set.find(type) != alltactics_set.end()) {
        out << DINDENT
            << "auto "
            << name
            << " = std::make_shared<"
            << type
            << ">(\""
            << name
            << "\", bb);"
            << std::endl;
    } else {
        out << DINDENT << "auto " << name << " = std::make_shared<" << type << ">(bb);" << std::endl;
    }
}

void BTBuilder::add_to_composite(std::string comp, std::string child) {
    out << DINDENT << comp << "->AddChild(" << child << ");" << std::endl;
}

void BTBuilder::set_decorator_child(std::string decorator, std::string child) {
    out << DINDENT << decorator << "->SetChild(" << child << ");" << std::endl;
}

void BTBuilder::defines(nlohmann::json jsonData) {
    NodeType type = determine_type(jsonData);

    switch (type) {
    case SELECTOR:
        define_sel(jsonData["title"]);
        for (auto child : jsonData["children"]) {
            defines(nodes[child]);
        }
        break;
    case SEQUENCE:
        define_seq(jsonData["title"], jsonData["name"], jsonData["properties"]);
        for (auto child : jsonData["children"]) {
            defines(nodes[child]);
        }
        break;
    case DECORATOR: {
        define_dec(jsonData["title"], jsonData["name"], jsonData);
        auto child = jsonData["child"];
        defines(nodes[jsonData["child"]]);
        break; }
    case LEAF:
        define_nod(jsonData["title"], jsonData["name"]);
        break;
    }

    // Add properties to private blackboard if needed
    std::unordered_map<std::string, json> properties = jsonData["properties"];
    if (properties.size() > 0) {
        for (auto property : properties) {
            if (property.second.is_string()) {
                std::string value = property.second.get<std::string>();

                if (value == "true") {
                    out << DINDENT
                        << jsonData["title"].get<std::string>()
                        << "->private_bb->SetBool(\""
                        << property.first
                        << "\", "
                        << property.second.get<std::string>()
                        << ");\n";
                } else if (value == "false") {
                    out << DINDENT
                        << jsonData["title"].get<std::string>()
                        << "->private_bb->SetBool(\""
                        << property.first
                        << "\", "
                        << property.second.get<std::string>()
                        << ");\n";
                } else {
                    out << DINDENT
                        << jsonData["title"].get<std::string>()
                        << "->private_bb->SetString(\""
                        << property.first
                        << "\", \""
                        << property.second.get<std::string>()
                        << "\");\n";
                }
            }

            if (property.second.is_number()) {
                std::string propertyType = "Double";
                if (property.second.is_number_integer()) {
                    propertyType = "Int";
                }

                out << DINDENT
                    << jsonData["title"].get<std::string>()
                    << "->private_bb->Set" << propertyType << "(\""
                    << property.first
                    << "\", ";

                if (property.second.is_number_integer()) {
                    out << (int) property.second.get<double>();
                } else {
                    out << property.second.get<double>();
                }
                
                out << ");\n";
            }

            if (property.second.is_boolean()) {
                out << DINDENT
                    << jsonData["title"].get<std::string>()
                    << "->private_bb->SetBool(\""
                    << property.first
                    << "\", "
                    << (property.second.get<bool>() ? "true" : "false")
                    << ");\n";
            }
        }
    }
}

void BTBuilder::build_structure(nlohmann::json root) {
    NodeType type = determine_type(root);
    switch (type) {
    case SELECTOR:
    case SEQUENCE:
        for (auto child : root["children"]) {
            add_to_composite(root["title"], nodes[child]["title"]);
            build_structure(nodes[child]);
        }
        break;
    case DECORATOR: {
        auto child = root["child"];
        set_decorator_child(root["title"], nodes[child]["title"]);
        build_structure(nodes[child]);
        break; }
    case LEAF:
        // NOP
        break;
    }
}

NodeType BTBuilder::determine_type(nlohmann::json json) {
    std::string name = json["name"];

    if (name == "Priority" || name == "MemPriority") {
        return SELECTOR;
    } else if (name == "Sequence"
            || name == "MemSequence"
            || name == "ParallelSequence"
            || name == "ParallelTactic") {
        return SEQUENCE;
    } else if (json.find("child") != json.end()) {
        return DECORATOR;
    }

    return LEAF;
}

}
