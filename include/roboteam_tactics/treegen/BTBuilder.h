#include <boost/optional.hpp>
#include <string>
#include <map>
#include <set>
#include <sstream>

#include "json.hpp"

namespace rtt {

/**
 * \enum NodeType
 * \brief The different types of nodes found in behavior trees
 */
enum NodeType { SEQUENCE, SELECTOR, DECORATOR, LEAF };

/**
 * \class BTBuilder
 * \brief Builds behavior trees from JSON files
 */
class BTBuilder {
    using json = nlohmann::json;

public:
    BTBuilder();
    ~BTBuilder();

    /**
     * \brief Does the actual building
     * \param json The JSON data, encoded by the nlohmann library
     * \param baseNamespace The namespace to put generated code in
     * \param namespaces Additional nested namespaces to use
     * \return An optional containing the generated code in string form if
     * the translation succeeded, or an empty optional if some error(s) occurred.
     */
    boost::optional<std::string> build(nlohmann::json json, std::string baseNamespace, std::vector<std::string> namespaces);

private:
    std::map<std::string, nlohmann::json> nodes;
    std::stringstream out;

    void define_seq(std::string name, std::string nodeType, json properties);
    void define_sel(std::string name);
    void define_dec(std::string name, std::string type, json data);
    void define_nod(std::string name, std::string type);

    void add_to_composite(std::string comp, std::string child);
    void set_decorator_child(std::string decorator, std::string child);

    void defines(nlohmann::json);
    void build_structure(nlohmann::json);

    std::string get_parallel_params_from_properties(json properties);

    NodeType determine_type(nlohmann::json);

    std::vector<std::string> allskills_list,
                             allconditions_list,
                             alltactics_list
                             ;
    std::set<std::string> allskills_set,
                          allconditions_set,
                          alltactics_set
                          ;
};

}
