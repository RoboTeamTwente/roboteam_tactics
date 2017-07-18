#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <map>
#include <queue>
#include <sys/resource.h>

namespace std {
    template<
        typename T
    >
    using reverse_priority_queue = priority_queue<T, std::vector<T>, std::greater<T>>;
}

namespace rtt {

template<
    typename T
>
int clamp(T const val, T const lo, T const hi) {
    return std::min(hi, std::max(val, lo));
}

struct Pos;

Pos operator+(Pos const & l, Pos const & r);

struct Pos {
    Pos() = default;
    Pos(Pos const &) = default;
    Pos(int x, int y) : x{x}, y{y} {}

    int x;
    int y;

    Pos normalize() {
        return { clamp(x, -1, 1), clamp(y, -1, 1) };
    }

    Pos rotate90() {
        return { -y, x };
    }

    Pos rotate270() {
        return { y, -x };
    }

    Pos rotateMin90() {
        return rotate270();
    }

    Pos rotate45() {
        return (this->rotate90() + *this).normalize();
    }

    Pos rotateMin45() {
        return (this->rotateMin90() + *this).normalize();
    }

    double length() const {
        return std::sqrt(x * x + y * y);
    }
} ;

Pos operator+(Pos const & l, Pos const & r) {
    return { l.x + r.x, l.y + r.y };
}

bool operator==(Pos const & l, Pos const & r) {
    return l.x == r.x && l.y == r.y;
}

bool operator!=(Pos const & l, Pos const & r) {
    return l.x != r.x || l.y != r.y;
}

bool operator< (Pos const &left, Pos const &right) {
    // if (left.x < right.x) {
        // return true;
    // } else if (left.x == right.x && left.y < right.y) {
        // return true;
    // } else {
        // return false;
    // }
    // =>
    return (left.x < right.x)
        || (left.x == right.x && left.y < right.y)
        ;
}

std::ostream& operator <<(std::ostream& stream, Pos const & p) {
    return stream << "{" << p.x << ", " << p.y << "}";
}

int horVertOrDiagDist(Pos const & l, Pos const & r) {
    // if (!(l.x == r.x) && !(l.y == r.y)) {
        // return std::abs(l.x - r.x);
    // }

    if (l.x == r.x) {
        return l.y - r.y;
    } 
        
    return l.x - r.x;
}

struct PosAndCost {
    PosAndCost() {}
    PosAndCost(Pos const & pos, int const cost) : pos{pos}, cost{cost} {}

    Pos pos;
    int cost;
} ;

template<
    typename T,
    typename U
>
std::set<T> keySet(std::map<T, U> const & m) {
    std::set<T> ts;

    for (auto const & entry : m) {
        ts.insert(entry.first);
    }

    return ts;
}

std::string getCmdOption(const std::vector<std::string>& args, const std::string & option) {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end() && (it + 1) != args.end()) {
        return *(it + 1);
    }

    return "";
}

bool cmdOptionExists(const std::vector<std::string>& args, const std::string& option) {
    return std::find(args.begin(), args.end(), option) != args.end();
}

auto block = [](std::string content) {
    int ww = content.size() + 2 + 2 * 4;

    std::string w;

    for (int i = 0; i < ww; ++i) {
        w += "-";
    }

    w += "\n";

    for (int i = 0; i < 4; ++i) {
        w += "-";
    }

    w += " ";

    w += content;

    w += " ";

    for (int i = 0; i < 4; ++i) {
        w += "-";
    }

    w += "\n";

    for (int i = 0; i < ww; ++i) {
        w += "-";
    }

    w += "\n";

    return w;
} ;

void setStackSize(uint64_t bytes) {
    const rlim_t kStackSize = bytes;
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "Setting stack size failed! setrlimit returned result = %d\n", result);
            }
        }
    }
}
    

} // namespace rtt
