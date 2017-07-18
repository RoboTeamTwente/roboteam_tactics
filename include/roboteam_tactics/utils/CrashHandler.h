#pragma once

#include <string>
#include <csignal>
#include <exception>

#define RESTART_EXIT_CODE 42

/*

These functions serve to restart an executable upon abnormal termination. The main purpose is to
prevent the whole system from dying during a match.

To use this functionality, you need to do two things:
  1. Run the executable with a script of this form:
         your_program "$@"
         while [ $? -e RESTART_EXIT_CODE ]; do
             your_program "$@"
         done
     your_program should be whatever executable you want to run (probably StrategyNode)
     $@ means 'all arguments passed to this script', i.e. the arguments get forwarded
     $? is the exit code of the last program to execute
     RESTART_EXIT_CODE is #defined in this file.
     
  2. In the main function of your executable, preferably as the very first statement, call registerAll().
     This will set crash handlers on std::quick_exit() and std::terminate() which will show an error message
     and then return the proper exit code (RESTART_EXIT_CODE).
*/

namespace rtt {
namespace crash{

void commonAlert(std::string source, bool restarting = true);
void restartSystem();
void atAbort(int);
void atQuickExit();
void atTerminate();
void atSegfault(int);
void registerAll();

}
}


