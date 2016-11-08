#pragma once

#include <string>
#include <unordered_map>
#include <memory>

#include "roboteam_msgs/StringEntry.h"
#include "roboteam_msgs/BoolEntry.h"
#include "roboteam_msgs/Int32Entry.h"
#include "roboteam_msgs/Float64Entry.h"
#include "roboteam_msgs/Blackboard.h"

namespace bt
{

class Blackboard
{
public:
    // Default constructors enabled
    Blackboard() = default;
    Blackboard(const Blackboard&) = default;
    Blackboard(Blackboard&&) = default;
    Blackboard& operator=(const Blackboard&) = default;
    Blackboard& operator=(Blackboard&&) = default;
    virtual ~Blackboard() = default;

    Blackboard(const roboteam_msgs::Blackboard &msg) {
        fromMsg(msg);
    }

    void SetBool(std::string key, bool value) { bools[key] = value; }
    bool GetBool(std::string key)
    {
        if (bools.find(key) == bools.end()) {
            bools[key] = false;
        }
        return bools[key];
    }
    bool HasBool(std::string key) const { return bools.find(key) != bools.end(); }

    void SetInt(std::string key, int value)  { ints[key] = value; }
    int GetInt(std::string key)
    {
        if (ints.find(key) == ints.end()) {
            ints[key] = 0;
        }
        return ints[key];
    }
    bool HasInt(std::string key) const  { return ints.find(key) != ints.end(); }

    void SetFloat(std::string key, float value)  { floats[key] = value; }
    float GetFloat(std::string key)
    {
        if (floats.find(key) == floats.end()) {
            floats[key] = 0.0f;
        }
        return floats[key];
    }
    bool HasFloat(std::string key) const  { return floats.find(key) != floats.end(); }

    void SetDouble(std::string key, double value)  { doubles[key] = value; }
    double GetDouble(std::string key)
    {
        if (doubles.find(key) == doubles.end()) {
            doubles[key] = 0.0f;
        }
        return doubles[key];
    }
    bool HasDouble(std::string key) const  { return doubles.find(key) != doubles.end(); }

    void SetString(std::string key, std::string value)  { strings[key] = value; }
    std::string GetString(std::string key)
    {
        if (strings.find(key) == strings.end()) {
            strings[key] = "";
        }
        return strings[key];
    }
    bool HasString(std::string key) const  { return strings.find(key) != strings.end(); }

    using Ptr = std::shared_ptr<Blackboard>;

    roboteam_msgs::Blackboard toMsg() {
        roboteam_msgs::Blackboard msg;

        for (const auto& i : bools) {
            roboteam_msgs::BoolEntry entry;
            entry.name = i.first;
            entry.value = i.second;
            msg.bools.push_back(entry);
        }

        for (const auto& i : doubles) {
            roboteam_msgs::Float64Entry entry;
            entry.name = i.first;
            entry.value = i.second;
            msg.doubles.push_back(entry);
        }

        for (const auto& i : strings) {
            roboteam_msgs::StringEntry entry;
            entry.name = i.first;
            entry.value = i.second;
            msg.strings.push_back(entry);
        }

        for (const auto& i : ints) {
            roboteam_msgs::Int32Entry entry;
            entry.name = i.first;
            entry.value = i.second;
            msg.ints.push_back(entry);
        }

        return msg;
    }

    void fromMsg(const roboteam_msgs::Blackboard &msg) {
        for (const roboteam_msgs::BoolEntry be : msg.bools) {
            SetBool(be.name, be.value);
        }

        for (const roboteam_msgs::StringEntry se : msg.strings) {
            SetString(se.name, se.value);
        }

        for (const roboteam_msgs::Int32Entry ie : msg.ints) {
            SetInt(ie.name, ie.value);
        }

        for (const roboteam_msgs::Float64Entry de : msg.doubles) {
            SetDouble(de.name, de.value);
        }
    }

    const std::unordered_map<std::string, bool> getBools() { return bools; }
    const std::unordered_map<std::string, int> getInts() { return ints; }
    const std::unordered_map<std::string, float> getFloats() { return floats; }
    const std::unordered_map<std::string, double> getDoubles() { return doubles; }
    const std::unordered_map<std::string, std::string> getStrings() { return strings; }
    
protected:
    std::unordered_map<std::string, bool> bools;
    std::unordered_map<std::string, int> ints;
    std::unordered_map<std::string, float> floats;
    std::unordered_map<std::string, double> doubles;
    std::unordered_map<std::string, std::string> strings;
};

}
