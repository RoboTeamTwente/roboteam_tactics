#include <algorithm>
#include <stack>
#include <string>
#include <unordered_map>

#include "roboteam_tactics/treegen/BTBuilder.h"
#include "roboteam_tactics/treegen/json.hpp"
#include "roboteam_tactics/generated/allconditions_set.h"
#include "roboteam_tactics/generated/allskills_set.h"
#include "roboteam_tactics/generated/alltactics_set.h"

#define INDENT "    "
#define DINDENT "        "

using json = nlohmann::json;

namespace rtt {

BTBuilder::BTBuilder() {}
BTBuilder::~BTBuilder() {}

std::string BTBuilder::build(nlohmann::json json) {
    std::stack<std::string> stack;
    stack.push(json["root"]);

    // Give all nodes who are not a skill nor condition
    // a unique name by appending a number at the end
    int ctr = 0;
    for (auto& element : json["nodes"]) {
        std::string title = element["title"];

        if (allskills_set.find(element["name"]) == allskills_set.end()
                && allconditions_set.find(element["name"]) == allconditions_set.end()) {
            // Get title and append ctr
            title += "_" + std::to_string(ctr++);
            // Replace spaces with _
        }

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

    out << INDENT << "bt::BehaviorTree make_"
        << json["title"].get<std::string>()
        << "(ros::NodeHandle n) {" 
        << std::endl; 
    out << DINDENT << "bt::BehaviorTree tree;" << std::endl;
    out << DINDENT << "auto bb = tree.GetBlackboard();" << std::endl;

    nlohmann::json root = nodes[json["root"]];
    defines(root);
    build_structure(root);

    out << DINDENT << "tree.SetRoot(" << root["title"].get<std::string>() << ");" << std::endl;
    out << DINDENT << "return tree;" << std::endl;
    out << INDENT << "}" << std::endl;

    return out.str();
}

void BTBuilder::define_seq(std::string name) {
    std::string memSeqName = "MemSequence";
    std::string parallelSeqName = "ParallelSequence";
    std::string type = "bt::Sequence";
    std::string params = "";

    if (name.compare(0, memSeqName.length(), memSeqName) == 0) {
        // It's a mem sequence
        type = "bt::MemSequence";
    } else if (name.compare(0, parallelSeqName.length(), parallelSeqName) == 0) {
        type = "bt::ParallelSequence";
        // Need all to succeed, one to fail. See ParallelSequence.hpp
        params = "true, false";
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
            << ">(n, \""
            << name
            << "\", bb);"
            << std::endl;
    } else if (allconditions_set.find(type) != allconditions_set.end()) {
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
        define_seq(jsonData["title"]);
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
            || name == "ParallelSequence") {
        return SEQUENCE;
    } else if (json.find("child") != json.end()) {
        return DECORATOR;
    }

    return LEAF;
}

}
