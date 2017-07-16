#pragma once

#include <array>
#include <random>
#include <cstring>

#include "utils.h"

namespace rtt {

template<
    typename T,
    unsigned int W,
    unsigned int H
>
class PropertyGrid {
public:
    static size_t constexpr NUM_ELEMS = W * H;

    PropertyGrid() {}
    PropertyGrid(T defaultValue) : defaultValue{defaultValue} 
            {
        clear();        
    }

    T& get(Pos const & p) {
        return elems[p.y * W + p.x];
    }

    T const & read(Pos const & p) const {
        return elems[p.y * W + p.x];
    }

    void clear() {
        std::memset(elems.data(), defaultValue, sizeof(T) * NUM_ELEMS);
    }

private:
    std::array<T, NUM_ELEMS> elems {};
    T const defaultValue {};
} ;

template<
    unsigned int W,
    unsigned int H
>
class Grid {
public:
    static size_t constexpr NUM_ELEMS = W * H;

    Grid() : mt(rd())
           {
        // TODO: Use appropriate array constructor?
        // std::memset(costs.data(), -1, sizeof(short) * NUM_ELEMS);
    }

    inline bool isOccupied(Pos const & pos) const {
        return obstacles[pos.y * W + pos.x];
    }

    inline bool isOccupiedSafe(Pos const & pos) const {
        return isInGrid(pos) and isOccupied(pos);
    }

    inline void setOccupied(Pos const & pos, bool occupied) {
        obstacles[pos.y * W + pos.x] = occupied;
    }

    inline void setOccupiedSafe(Pos const & pos, bool occupied) {
        if (isInGrid(pos)) {
            setOccupied(pos, occupied);
        }
    }

    inline bool isInGrid(Pos const & pos) const {
        return pos.x >= 0
            and pos.y >= 0
            and static_cast<unsigned int>(pos.x) < W
            and static_cast<unsigned int>(pos.y) < H
            ;
    }

    void setOccupiedRect(Pos const & blPos, Pos const & trPos) {
        for (int x = blPos.x; x < trPos.x + 1; ++x) {
            setOccupied(x, blPos.y, true);
            setOccupied(x, trPos.y, true);
        }

        for (int y = blPos.y + 1; y < trPos.y; ++y) {
            setOccupied(blPos.x, y, true);
            setOccupied(trPos.x, y, true);
        }
    }

    void setOccupiedCircle(Pos const & pos0, int const rad) {
        int x0 = pos0.x;
        int y0 = pos0.y;
        int x = rad;
        int y = 0;
        int err = 0;

        while (x >= y) {
            setInQuadrant(x0, y0, x, y, true);

            y += 1;

            if (err <= 0) {
                err += 2*y + 1;
            }

            if (err > 0) {
                int oldX = x;

                x -= 1;
                err -= 2*x + 1;

                // int newX = x
                int newY = y;
                // int oldY = y - 1

                setInQuadrant(x0, y0, oldX, newY, true);
                // setInQuadrant(x0, y0, newX, oldY, true)
            }
        }
    }

    void getNeighbours(Pos const & pos, std::vector<Pos> & out) const {
        out.clear();

        Pos d {1, 0};
        for (int i = 0; i < 8; ++i) {
            auto const newPos = pos + d;
            if (isInGrid(newPos) && !isOccupied(newPos)) {
                out.push_back(newPos);
            }
            d = d.rotate45();
        }
    }

    void getNeighboursAndCost(Pos const & pos, std::vector<PosAndCost> & out) const {
        out.clear();

        Pos d {1, 0};

        for (int i = 0; i < 4; ++i) {
            auto const newPos = pos + d;

            if (isInGrid(newPos) && !isOccupied(newPos)) {
                out.emplace_back(newPos, 10);
            }

            d = d.rotate90();
        }

        d = {1, 1};

        for (int i = 0; i < 4; ++i) {
            auto const newPos = pos + d;

            if (isInGrid(newPos) && !isOccupied(newPos)) {
                out.emplace_back(newPos, 14);
            }

            d = d.rotate90();
        }
    }

    void getNeighboursAndCost(Pos const & pos, std::array<PosAndCost, 9> & out, int & outSize) const {
        Pos d {1, 0};
        int outI = 0;

        for (int i = 0; i < 4; ++i) {
            auto const newPos = pos + d;

            if (isInGrid(newPos) && !isOccupied(newPos)) {
                out[outI++] = PosAndCost(newPos, 10);
            }

            d = d.rotate90();
        }

        d = {1, 1};

        for (int i = 0; i < 4; ++i) {
            auto const newPos = pos + d;

            if (isInGrid(newPos) && !isOccupied(newPos)) {
                out[outI++] = PosAndCost(newPos, 14);
            }

            d = d.rotate90();
        }

        outSize = outI;
    }

    inline void setInQuadrant( int const x0
                             , int const y0
                             , int const x
                             , int const y
                             , bool occupied
                             ) {
        setOccupiedSafe({x0 + x, y0 + y}, occupied);
        setOccupiedSafe({x0 + y, y0 + x}, occupied);
        setOccupiedSafe({x0 + x, y0 - y}, occupied);
        setOccupiedSafe({x0 + y, y0 - x}, occupied);
        setOccupiedSafe({x0 - x, y0 + y}, occupied);
        setOccupiedSafe({x0 - y, y0 + x}, occupied);
        setOccupiedSafe({x0 - x, y0 - y}, occupied);
        setOccupiedSafe({x0 - y, y0 - x}, occupied);
    }

    void fillRandomly(int const n) {
        std::uniform_int_distribution<int> dist(0, NUM_ELEMS - 1);

        for (int i = 0; i < n; i++) {
            int p = dist(mt);
            obstacles[p] = true;
        }
    }

    void fillRandomRobots(int n, int s) {
        std::uniform_int_distribution<int> xDist(0, W);
        std::uniform_int_distribution<int> yDist(0, H);
        for (int i = 0; i < n; ++i) {
            int centerX = xDist(mt);
            int centerY = yDist(mt);

            placeRobot({centerX, centerY}, s);
        }
    }

    void placeRobot(Pos const & pos, int rad) {
        std::cout << "Placing robot at: " << pos << "\n";
        setOccupiedCircle(pos, rad);
    }

    void clear() {
        std::memset(obstacles.data(), 0, sizeof(bool) * NUM_ELEMS);
        // std::memset(costs.data(), -1, sizeof(short) * NUM_ELEMS);
        // std::memset(cameFroms.data(), 0, sizeof(Pos) * NUM_ELEMS);
    }

    void diagonalBarrier(int x, int y, int s, int n) {
        int totalWidth = (n - 1) * (s / 2);
        int left = x - totalWidth / 2;
        int top = y + totalWidth / 2;

        for (int i = 0; i < n; ++i) {
            placeRobot({left + i * (s / 2), top - i * (s / 2)}, s);
        }
    }

    std::string toString(int gridScale = -1, std::vector<Pos> const & path = {}, std::set<Pos> const & visitedPositions = {}) {
        std::string result;
        result.reserve((W + 1) * H);

        std::set<Pos> const pathSet(path.cbegin(), path.cend());

        for (int y = H - 1; y > -1; --y) {
            for (int x = 0; x < static_cast<int>(W); ++x) {
                if (isOccupied({x, y})) {
                    result += "#";
                } else if (pathSet.find({x, y}) != pathSet.end()) {
                    result += "-";
                } else if (gridScale != -1
                        && (((x % gridScale) == 0) || ((y % gridScale) == 0))
                        && (x > 0 && y > 0)) {
                    result += "+";
                } else if (visitedPositions.find({x, y}) != visitedPositions.end()) {
                    result += "$";
                } else {
                    result += " ";
                }
            }

            result += "\n";
        }

        return result;
    }

private:
    // IDEA: Maybe the bottom info can be packed 
    // into a long long? 
    std::array<bool, NUM_ELEMS> obstacles {};

    std::random_device rd;
    std::mt19937 mt;

} ;

} // namespace rtt
