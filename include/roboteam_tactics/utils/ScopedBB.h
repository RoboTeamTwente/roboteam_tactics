#pragma once

#include "roboteam_tactics/bt.hpp"

namespace rtt {

class ScopedBB {
public:
   ScopedBB(bt::Blackboard& dataBB, std::string scope);
   // Destructor is to prevent copy constructors et al.
   ~ScopedBB();

   ScopedBB& setBool(std::string key, bool value);
   ScopedBB& setInt(std::string key, int value);
   ScopedBB& setDouble(std::string key, double value);
   ScopedBB& setString(std::string key, std::string value);

   bool        getBool(std::string key);
   int         getInt(std::string key);
   double      getDouble(std::string key);
   std::string getString(std::string key);

   bool hasBool(std::string key);
   bool hasInt(std::string key);
   bool hasDouble(std::string key);
   bool hasString(std::string key);

private:
   bt::Blackboard& dataBB;
   std::string prefix;
} ;

} // rtt
