#pragma once

#include "roboteam_tactics/bt.hpp"

namespace rtt {

/**
 * \class ScopedBB
 * \brief Provides a convenient interface for building blackboards with a prefix
 *
 * Usage:
 *
 * bt::Blackboard bb;
 * rtt::ScopedBB(bb, "GetBall_")
 *      .setDouble("targetAngle", lookAng)
 *      .setBool("passOn", false)
 *      ;
 */
class ScopedBB {
public:
   /**
    * \brief Constructor.
    * @param dataBB The blackboard to use in the background. This blackboard will store all
    * data, and it may be modified. 
    * @param scope The prefix to apply to all new parameters
    */
   ScopedBB(bt::Blackboard& dataBB, std::string scope);
   // Destructor is to prevent copy constructors et al.
   ~ScopedBB();
   
   /*
    * All methods below take the prefix into account
    */

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
