#include <vector>

#include "Grid.h"
#include "utils.h"

namespace rtt {

template<
    unsigned int W,
    unsigned int H
>
std::tuple<std::vector<Pos>, std::set<Pos>> dijkstra(Grid<W, H> const & grid, Pos const & start, Pos const & end) {
    std::reverse_priority_queue<std::tuple<int, Pos>> frontier;
    frontier.push(std::make_tuple(0, start));

    std::map<Pos, int> cost;
    std::map<Pos, Pos> cameFrom;

    cost[start] = 0;

    std::vector<PosAndCost> neighboursAndCosts;
    neighboursAndCosts.reserve(9);

    while (!frontier.empty()) {
        auto current = std::get<1>(frontier.top());
        frontier.pop();

        if (current == end) {
            std::vector<Pos> path;
            path.reserve(2*(W + H));

            while (current != start) {
                path.push_back(current);
                current = cameFrom[current];
            }

            path.push_back(current);
            std::reverse(path.begin(), path.end());
            
            return std::make_tuple(path, keySet(cost));
        }

        auto currentCost = cost[current];

        grid.getNeighboursAndCost(current, neighboursAndCosts);
        for (auto const & nac : neighboursAndCosts) {
            auto newCost = currentCost + nac.cost;
            auto costIt = cost.find(nac.pos);
            if (costIt == cost.end() || newCost < costIt->second) {
                cost[nac.pos] = newCost;
                // frontier.push(std::make_tuple(newCost, nac.pos));
                frontier.emplace(newCost, nac.pos);
                cameFrom[nac.pos] = current;
            }
        }
    }

    return std::make_tuple<std::vector<Pos>, std::set<Pos>>({}, keySet(cost));
}

} // namespace rtt
