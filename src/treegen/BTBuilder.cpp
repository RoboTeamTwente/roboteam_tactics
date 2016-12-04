#include <algorithm>
#include <stack>
#include <string>
#include <unordered_map>

#include "roboteam_tactics/treegen/BTBuilder.h"
#include "roboteam_tactics/treegen/json.hpp"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#define INDENT "    "
#define DINDENT "        "

using json = nlohmann::json;

namespace rtt {

BTBuilder::BTBuilder() {}
BTBuilder::~BTBuilder() {}

std::string BTBuilder::build(nlohmann::json json) {
    namespace f = rtt::factories;

    allskills_list = f::get_entry_names<Skill>();
    allconditions_list = f::get_entry_names<Condition>();
    alltactics_list = f::get_entry_names<Tactic>();

    // To turn every list into a set and clear the previous sets
    auto initializeSet = [](std::set<std::string> &theSet, std::vector<std::string> &theVector) {
        theSet.clear();
        theSet.insert(theVector.begin(), theVector.end());
    };
    
    initializeSet(allskills_set, allskills_list);
    initializeSet(allconditions_set, allconditions_list);
    initializeSet(alltactics_set, alltactics_list);

    std::stack<std::string> stack;
    stack.push(json["root"]);

    std::set<std::string> usedSkills, usedConditions, usedTactics;

    using namespace rtt::factories;
    auto& skillRepo = getRepo<Factory<Skill>>();
    auto& conditionRepo = getRepo<Factory<Condition>>();
    auto& tacticRepo = getRepo<Factory<Tactic>>();

    // Give all nodes who are not a skill nor condition
    // a unique name by appending a number at the end
    int ctr = 0;
    for (auto& element : json["nodes"]) {
        std::string title = element["title"];

        if (skillRepo.find(element["name"].get<std::string>()) != skillRepo.end()) {
            usedSkills.insert(element["name"].get<std::string>());
        }

        if (conditionRepo.find(element["name"].get<std::string>()) != conditionRepo.end()) {
            usedConditions.insert(element["name"].get<std::string>());
        }

        if (tacticRepo.find(element["name"].get<std::string>()) != tacticRepo.end()) {
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
        // out << INDENT << "// " << skill << "\n";
        out << INDENT << "#include \"roboteam_tactics/skills/" << skill << ".h\"\n";
    }

    out << INDENT << "// Used conditions: \n";
    for (const auto& condition : usedConditions) {
        // out << INDENT << "// " << condition << "\n";
        out << INDENT << "#include \"roboteam_tactics/conditions/" << condition << ".h\"\n";
    }

    out << INDENT << "// Used tactics: \n";
    for (const auto& tactic : usedTactics) {
        // out << INDENT << "// " << tactic << "\n";
        out << INDENT << "#include \"roboteam_tactics/tactics/" << tactic << ".h\"\n";
    }
    out << "\n";

    // Create constructor function
    out << INDENT << "namespace rtt {\n\n";
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
    out << INDENT << "} // rtt\n\n";

    out << INDENT << "namespace {\n\n";
    out << INDENT
        << "rtt::factories::TreeRegisterer rtt_"
        << json["title"].get<std::string>()
        << "_registerer(\""
        << json["title"].get<std::string>()
        << "\", &rtt::make_"
        << json["title"].get<std::string>()
        << ");\n\n";
    out << INDENT << "} // anonymous namespace\n";

    return out.str();
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
        std::string successOnAll = properties["successOnAll"].get<bool>() ? "true" : "false";
        std::string failOnAll = properties["failOnAll"].get<bool>() ? "true" : "false";
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

void BTBuilder::define_dec(std::string name, std::string type) {
    std::string params = "";

    if (type == "Repeat") {
        type = "bt::Repeater";
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
        define_dec(jsonData["title"], jsonData["name"]);
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
                out << DINDENT
                    << jsonData["title"].get<std::string>()
                    << "->private_bb->SetString(\""
                    << property.first
                    << "\", \""
                    << property.second.get<std::string>()
                    << "\");\n";
            }

            if (property.second.is_number()) {
                out << DINDENT
                    << jsonData["title"].get<std::string>()
                    << "->private_bb->SetDouble(\""
                    << property.first
                    << "\", "
                    << property.second.get<double>()
                    << ");\n";
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
