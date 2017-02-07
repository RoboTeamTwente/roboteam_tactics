#include "roboteam_tactics/utils/ScopedBB.h"

namespace rtt {

ScopedBB::ScopedBB(bt::Blackboard& dataBB, std::string scope)
    : dataBB{dataBB} {
    if (!scope.empty()) {
        prefix = scope + "_";
    }
}

ScopedBB::~ScopedBB() { }

ScopedBB& ScopedBB::setBool(std::string key, bool value) {
    dataBB.SetBool(prefix + key, value);
    return *this;
}

ScopedBB& ScopedBB::setInt(std::string key, int value) {
    dataBB.SetInt(prefix + key, value);
    return *this;
}

ScopedBB& ScopedBB::setDouble(std::string key, double value) {
    dataBB.SetDouble(prefix + key, value);
    return *this;
}

ScopedBB& ScopedBB::setString(std::string key, std::string value) {
    dataBB.SetString(prefix + key, value);
    return *this;
}

bool ScopedBB::getBool(std::string key) {
    return dataBB.GetBool(prefix + key);
}

int ScopedBB::getInt(std::string key) {
    return dataBB.GetInt(prefix + key);
}

double ScopedBB::getDouble(std::string key) {
    return dataBB.GetDouble(prefix + key);
}

std::string ScopedBB::getString(std::string key) {
    return dataBB.GetString(prefix + key);
}

bool ScopedBB::hasBool(std::string key) {
    return dataBB.HasBool(prefix + key);
}

bool ScopedBB::hasInt(std::string key) {
    return dataBB.HasInt(prefix + key);
}

bool ScopedBB::hasDouble(std::string key) {
    return dataBB.HasDouble(prefix + key);
}

bool ScopedBB::hasString(std::string key) {
    return dataBB.HasString(prefix + key);
}

} // rtt
