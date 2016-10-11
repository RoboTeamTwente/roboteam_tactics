#include <stack>

#include "roboteam_tactics/treegen/BTBuilder.h"
#include "roboteam_tactics/treegen/json.hpp"

#define INDENT "    "
#define DINDENT "        "

BTBuilder::BTBuilder() {}
BTBuilder::~BTBuilder() {}

std::string BTBuilder::build(nlohmann::json json) {
    std::stack<std::string> stack;
    stack.push(json["root"]);
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

    out << INDENT << "bt::BehaviorTree build_tree() {" << std::endl; 
    out << DINDENT << "bt::BehaviorTree tree;" << std::endl;
    out << DINDENT << "auto& bb = tree.GetBlackboard();" << std::endl;

    nlohmann::json root = nodes[json["root"]];
    defines(root);
    build_structure(root);

    out << DINDENT << "tree.SetRoot(" << root["title"] << ");" << std::endl;
    out << DINDENT << "return tree;" << std::endl;
    out << INDENT << "}" << std::endl;

    return out.str();
}

void BTBuilder::define_seq(std::string name) {
    out << DINDENT << "auto " << name << " = std::make_shared<bt::Sequence>();" << std::endl;
}

void BTBuilder::define_sel(std::string name) {
    out << DINDENT << "auto " << name << " = std::make_shared<bt::Selector>();" << std::endl;
}

void BTBuilder::define_dec(std::string name, std::string type) {
    out << DINDENT << "auto " << name << " = std::make_shared<" << type << ">(bb);" << std::endl;
}
void BTBuilder::define_nod(std::string name, std::string type) {
    out << DINDENT << "auto " << name << " = std::make_shared<" << type << ">(bb);" << std::endl;
}

void BTBuilder::add_to_composite(std::string comp, std::string child) {
    out << DINDENT << comp << "->AddChild(" << child << ");" << std::endl;    
}

void BTBuilder::set_decorator_child(std::string decorator, std::string child) {
    out << DINDENT << decorator << "->SetChild(" << child << ");" << std::endl;
}

void BTBuilder::defines(nlohmann::json json) {
    NodeType type = determine_type(json);
    switch (type) {
    case SELECTOR:
        define_sel(json["title"]);
        for (auto child : json["children"]) {
            defines(nodes[child]);
        }
        break;
    case SEQUENCE:
        define_seq(json["title"]);
        for (auto child : json["children"]) {
            defines(nodes[child]);
        }
        break;
    case DECORATOR: {
        define_dec(json["title"], json["name"]);
        auto child = json["child"];
        defines(nodes[json["child"]]);
        break; }
    case LEAF:

        define_nod(json["title"], json["name"]);
        break;
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
    } else if (name == "Sequence" || name == "MemSequence") {
        return SEQUENCE;
    } else if (json.find("child") != json.end()) {
        return DECORATOR;
    }
    return LEAF;
}
