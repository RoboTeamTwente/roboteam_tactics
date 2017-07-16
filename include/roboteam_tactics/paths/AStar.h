#include <queue>
#include <set>
#include <limits>
#include <chrono>

#include "utils.h"
#include "Grid.h"

namespace rtt {

using CostFunc = int(*)(Pos const &, Pos const &);

// Doggedly slow
int asTheCrowFlies(Pos const & start, Pos const & end) {
    auto dx = start.x - end.x;
    auto dy = start.y - end.y;
    return std::sqrt(dx * dx + dy + dy) * 10;
}

int manhattan(Pos const & start, Pos const & end) {
    auto dx = std::abs(start.x - end.x);
    auto dy = std::abs(start.y - end.y);
    return dx + dy;
}

// Diagonal Dist
// function heuristic(node) =
    // dx = abs(node.x - goal.x)
    // dy = abs(node.y - goal.y)
    // return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
int diagonalDist(Pos const & start, Pos const & end) {
    int dx = std::abs(start.x - end.x);
    int dy = std::abs(start.y - end.y);

    constexpr int D = 10;
    constexpr int D2 = 14;

    return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
}

template <
    typename T
>
class PriorityBucketQueue {
private:
    struct Bucket {
        int priority;
        std::vector<T> units;
    } ;

    std::vector<Bucket> buckets;

    Bucket& makeOrGetBucket(int priority) {
        if (buckets.size() == 0) {
            buckets.emplace_back();
            buckets.back().priority = priority;
            return buckets.back();
        } 

        auto it = buckets.begin();

        for (; it != buckets.end(); it++) {
            if (priority == it->priority) {
                return *it;
            } else if (priority > it->priority) {
                break;
            }
        }

        it = buckets.emplace(it);
        it->priority = priority;

        return *it;
    }

public:
    PriorityBucketQueue() {
        buckets.reserve(20);
    }

    void push(int priority, T const & unit) {
        Bucket& bucket = makeOrGetBucket(priority);
        bucket.units.push_back(unit);
    }

    int empty() {
        return buckets.size() == 0;
    }

    T pop() {
        auto & bucket = buckets.back();

        T theUnit = bucket.units.back();
        bucket.units.pop_back();

        if (bucket.units.size() == 0) {
            buckets.pop_back();
        }

        return theUnit;
    }

    void printInternals() {
        std::cout << "Num buckets: " << buckets.size() << "\n";
        for (auto const & bucket : buckets) {
            std::cout << "Bucket size: " << bucket.units.size() << ", priority: " << bucket.priority << "\n";
        }
    }

} ;

template <
    typename T
>
class ReversePriorityQueue {
public:
    std::reverse_priority_queue<std::tuple<int, T>> queue;

    void push(int priority, T const & unit) {
        queue.push(std::make_tuple(priority, unit));
    }

    T pop() {
        T unit = std::get<1>(queue.top());
        queue.pop();
        return unit;
    }

    bool empty() {
        return queue.size() == 0;
    }
} ;

template<
    unsigned int W,
    unsigned int H
>
std::vector<Pos> reconstructPath(Pos const & start, Pos const & end, PropertyGrid<Pos, W, H> const & cameFrom) {
    std::vector<Pos> path;
    path.reserve(2*(W + H));

    Pos current = end;

    while (current != start) {
        path.push_back(current);
        current = cameFrom.read(current);
    }

    path.push_back(current);
    std::reverse(path.begin(), path.end());

    return path;
}

template <
    CostFunc costFunc,
    template<class> class PriorityQueue,
    bool BREAK_TIES,
    bool GET_VISITED_SET,
    unsigned int W,
    unsigned int H,
    unsigned int MAX_MS = std::numeric_limits<unsigned int>::max()
>
std::tuple<std::vector<Pos>, std::set<Pos>> aStar(Grid<W, H> const & grid, Pos const & start, Pos const & end) {
    using namespace std::chrono;

    PriorityQueue<Pos> frontier;
    frontier.push(0, start);

    PropertyGrid<short, W, H> cost;
    PropertyGrid<Pos, W, H> cameFrom;
    PropertyGrid<bool, W, H> visited;
    std::set<Pos> visitedSet;

    cost.get(start) = 0;

    constexpr float hGain = BREAK_TIES ? 2 : 1;
    std::array<PosAndCost, 9> neighboursAndCosts;

    int iterations = 0;
    using Clock = steady_clock;
    auto startTime = Clock::now();

    while (!frontier.empty()) {
        iterations++;

        auto current = frontier.pop();

        // If we've reached the end, reconstruct the path and return it
        if (current == end) {
            std::vector<Pos> path = reconstructPath(start, end, cameFrom);

            if (GET_VISITED_SET) {
                return std::make_tuple(path, visitedSet);
            } else {
                return std::make_tuple(path, std::set<Pos>());
            }
        }

        // Every 1000 iterations, if too much time has passed we exit early
        // and construct the best possible path we have
        if (iterations % 1000 == 0) {
            if ((Clock::now() - startTime) >= milliseconds(MAX_MS)) {
                std::vector<Pos> path = reconstructPath(start, current, cameFrom);
                
                // Last trajectory is just a straight line to the end because of early exit!
                path.push_back(end);

                if (GET_VISITED_SET) {
                    return std::make_tuple(path, visitedSet);
                } else {
                    return std::make_tuple(path, std::set<Pos>());
                }
            }
        }

        // Else expand the search from the current node
        auto currentCost = cost.get(current);
        int numNeighbours = 0;
        grid.getNeighboursAndCost(current, neighboursAndCosts, numNeighbours);

        for (int i = 0; i < numNeighbours; ++i) {
            auto const & neighbourAndCost = neighboursAndCosts[i];
            auto newCost = currentCost + neighbourAndCost.cost;

            if (!visited.get(neighbourAndCost.pos)
                    || newCost < cost.get(neighbourAndCost.pos)
                    ) {
                cost.get(neighbourAndCost.pos) = newCost;

                if (GET_VISITED_SET) {
                    visitedSet.insert(neighbourAndCost.pos);
                }

                int h = costFunc(neighbourAndCost.pos, end) * hGain;
                frontier.push(newCost + h, neighbourAndCost.pos);

                cameFrom.get(neighbourAndCost.pos) = current;
                visited.get(neighbourAndCost.pos) = true;
            }
        }
    }

    if (GET_VISITED_SET) {
        return std::make_tuple(std::vector<Pos>(), visitedSet);
    } else {
        return std::make_tuple(std::vector<Pos>(), std::set<Pos>());
    }
}

} // namespace rtt
