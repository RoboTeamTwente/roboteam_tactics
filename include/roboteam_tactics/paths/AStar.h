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
        struct Elem {
            T unit;
            bool valid;
        } ;

        int priority;
        std::vector<Elem> elems;
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

    int elemCount;

public:
    PriorityBucketQueue() {
        buckets.reserve(20);
        elemCount = 0;
    }

    int push(int priority, T const & unit) {
        Bucket& bucket = makeOrGetBucket(priority);
        bucket.elems.push_back({unit, true});

        ++elemCount;

        return bucket.elems.size() - 1;
    }

    int empty() {
        // return buckets.size() == 0;
        return elemCount == 0;
    }

    T pop() {
        if (elemCount == 0) {
            std::cout << "ERROR: elemCount == 0!\n";
            return {};
        }

        typename Bucket::Elem elem;

        do {
            auto & bucket = buckets.back();

            elem = bucket.elems.back();
            bucket.elems.pop_back();

            if (bucket.elems.size() == 0) {
                buckets.pop_back();
            }
        } while (!elem.valid);

        elemCount--;

        return elem.unit;
    }

    void lazyDelete(int priority, int index) {
        for (auto & bucket : buckets) {
            if (bucket.priority == priority) {
                bucket.elems[index].valid = false;
                elemCount--;
                break;
            } 
        }
    }

    void deleteUnit(T const & unit) {
        for (auto & bucket : buckets) {
            auto & elems = bucket.elems;
            elems.erase(
                    std::remove_if(elems.begin(),
                                   elems.end(),
                                   [&](auto elem) { return elem.unit == unit; }),
                    elems.end());
        }

        buckets.erase(
                std::remove_if(
                    buckets.begin(),
                    buckets.end(),
                    [](Bucket const & bucket) { return bucket.elems.size() == 0; }),
                buckets.end());
    }

    void deleteUnitWithHint(T const & unit, int knownPriority) {
        auto it = buckets.begin();

        for (; it != buckets.end(); ++it) {
            if (it->priority == knownPriority) {
                auto & elems = it->elems;
                elems.erase(
                        std::remove_if(elems.begin(),
                                       elems.end(),
                                       [&](auto elem) { return elem.unit == unit; }),
                        elems.end());
                break;
            }
        }

        if (it != buckets.end() && it->elems.size() == 0) {
            it = buckets.erase(it);
        }
    }

    std::vector<typename Bucket::Elem> const & getPriorityBucket() {
        return buckets.back().elems;
    }

    void printInternals() {
        std::cout << "Num buckets: " << buckets.size() << "\n";
        for (auto const & bucket : buckets) {
            std::cout << "Bucket size: " << bucket.elems.size() << ", priority: " << bucket.priority << "\n";
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

// A cool prime I found somewhere
int rngState = 104729;

// Slightly modified but
// inspired by: https://stackoverflow.com/questions/26237419/faster-than-rand
inline int fastRand(void) {
    rngState = (214013 * rngState + 2531011);
    return (rngState >> 16) & 0x7FFF;
}

// Slightly modified but
// inspired by: https://stackoverflow.com/questions/26237419/faster-than-rand
inline int fastRandArg(int state) {
    return ((214013 * state + 2531011) >> 16) & 0x7FFF;
}

inline int getRandBelow(int n) {
    // return xorshift32(&rngState) % n;
    return fastRand() % n;
}

/**
 * Works, but is too slow since swapping all the 
 * pos's throughout the array increase the runtime to 30 ms which is too long.
 */
template<
    unsigned int W,
    unsigned int H
>
inline void getNeighboursAndCostRandomized(Grid<W, H> const & grid, Pos const & pos, std::array<PosAndCost, 9> & out, int & outSize) {
    grid.getNeighboursAndCost(pos, out, outSize);

    for (int i = 0; i < (outSize - 1); ++i) {
        int swapWith = i + 1 + getRandBelow(outSize - 1 - i);
        std::swap(out[i], out[swapWith]);
    }
}

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

/**
 *  Followed the implementation on Amit's A* website/blog.
 */
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

    PriorityQueue<Pos> open;
    open.push(0, start);

    // Closed and isOpen need to be zeroed out, to make sure
    // that the if's in the for loop below give accurate results.
    // If they are not zeroed out then the algorithm might conclude
    // that a cell is on the closed or open list while in fact it isn't.
    PropertyGrid<bool, W, H> closed(false);
    PropertyGrid<bool, W, H> isOnOpen(false);

    // The rest is all set directly after the position closed is added
    // to the open or closed list. Therefore they do not need to be initialized.
    PropertyGrid<short, W, H> cost;
    PropertyGrid<Pos, W, H> cameFrom;
    PropertyGrid<int, W, H> bucketPriority;
    PropertyGrid<int, W, H> bucketIndex;

    std::set<Pos> visitedSet;

    cost.get(start) = 0;

    constexpr float hGain = BREAK_TIES ? 1.2 : 1;
    std::array<PosAndCost, 9> neighboursAndCosts;

    int iterations = 0;
    using Clock = steady_clock;
    auto startTime = Clock::now();

    while (!open.empty()) {
        iterations++;

        auto current = open.pop();
        closed.get(current) = true;

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
                auto const & elems = open.getPriorityBucket();

                // Base case, we keep the current popped element
                Pos minPos = current;
                int minH = costFunc(current, end) * hGain;

                for (auto const & elem : elems) {
                    if (!elem.valid) continue;
                    auto const & candidatePos = elem.unit;

                    int candidateH = costFunc(candidatePos, end) * hGain;

                    if (candidateH < minH) {
                        minPos = candidatePos;
                        minH = candidateH;
                    }
                }

                std::vector<Pos> path = reconstructPath(start, minPos, cameFrom);
                
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

        // Reorders the order of the positions sometimes, causing A* to favor
        // a fairly straight line instead of an angular one.
        grid.getNeighboursAndCostRandomlyOrdered(fastRandArg(current.x + current.y) % 2, current, neighboursAndCosts, numNeighbours);

        for (int i = 0; i < numNeighbours; ++i) {
            auto const & neighbourAndCost = neighboursAndCosts[i];

            auto const & neighbour = neighbourAndCost.pos;
            auto newCost = currentCost + neighbourAndCost.cost;

            if (isOnOpen.read(neighbour) && newCost < cost.read(neighbour)) {
                open.lazyDelete(bucketPriority.read(neighbour), bucketIndex.read(neighbour));
                isOnOpen.get(neighbour) = false;
            }

            if (closed.read(neighbour) && newCost < cost.read(neighbour)) {
                closed.get(neighbour) = false;
            }

            if (!isOnOpen.get(neighbour) && !closed.get(neighbour)) {
                cost.get(neighbour) = newCost;
                isOnOpen.get(neighbour) = true;
                
                if (GET_VISITED_SET) {
                    visitedSet.insert(neighbour);
                }

                int h = costFunc(neighbour, end) * hGain;
                int neighbourBucketIndex = open.push(newCost + h, neighbour);

                bucketPriority.get(neighbour) = newCost + h;
                bucketIndex.get(neighbour) = neighbourBucketIndex;

                cameFrom.get(neighbour) = current;
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
