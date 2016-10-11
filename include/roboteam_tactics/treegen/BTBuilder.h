#include "json.hpp"
#include <string>
#include <map>
#include <sstream>

enum NodeType { SEQUENCE, SELECTOR, DECORATOR, LEAF };

class BTBuilder {
public:
    BTBuilder();
    ~BTBuilder();
    std::string build(nlohmann::json);
private:
    std::map<std::string, nlohmann::json> nodes;
    std::stringstream out;

    void define_seq(std::string name);
    void define_sel(std::string name);
    void define_dec(std::string name, std::string type);
    void define_nod(std::string name, std::string type);

    void add_to_composite(std::string comp, std::string child);
    void set_decorator_child(std::string decorator, std::string child);

    void defines(nlohmann::json);
    void build_structure(nlohmann::json);

    NodeType determine_type(nlohmann::json);
};

