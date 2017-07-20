#include "roboteam_tactics/utils/CrashHandler.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>
#include <ctime>
#include <csignal>
#include <execinfo.h>

namespace rtt {
namespace crash {

void writeLog(std::string source, bool hasException) {
	std::stringstream ss;
	ss << CRASH_LOG_DIR << "/rtt_crashlog-" << time(0) << ".log";

	std::ofstream log(ss.str().c_str());
	std::cerr << "Printing error log to " << ss.str() << "\n";

	log << "**** roboteam_tactics CRASH ****\n\n";
	log << "Executable: ";

	char exe[1024];
	int len = readlink("/proc/self/exe", exe, 1023);
	std::string exeName;
	if (len == -1) {
		exeName = std::string("<unable to retrieve>");
	} else {
		exe[len] = 0;
		exeName = std::string(exe);
	}

	log << exeName << "\n";
	log << "Source: " << source << "\n";
	if (hasException) {
		std::exception_ptr eptr = std::current_exception();
		if (eptr) {
			try {
				// It's impossible to dereference an exception_ptr manually, this is the correct method somehow.
				std::rethrow_exception(eptr);
				log << "<failed to print exception message>\n";
			} catch (const std::exception& e) {
				log << "Message: " << e.what() << "\n";
			}
		} else {
			log << "<no exception stored in std::current_exception()>\n";
		}
	}

	void* stacktraceElements[10];
	size_t st_size = backtrace(stacktraceElements, 10);
	char** stacktrace = backtrace_symbols(stacktraceElements, st_size);

	if (st_size > 0) {
		log << "Stack Trace: (probably useless...)\n";
	}

	for (size_t i = 0; i < st_size; i++) {
		log << stacktrace[i] << "\n";
	}

	log.flush();
	log.close();
}

void commonAlert(std::string source, bool restarting) {
	std::cerr << "\n\n\n\n\n\n\n\n\n\n" << std::endl;
	std::cerr << "***********************************CRASH***********************************\n" << std::endl;
	std::cerr << "*************************** The program crashed! **************************\n" << std::endl;
	std::cerr << "******************** Cause: ";
	std::cerr << source.c_str();
	std::cerr << "  ****************************\n"<< std::endl;
	if (restarting) {
		std::cerr << "*********************** The program will now restart **********************\n"<< std::endl;
	} else {
		std::cerr << "*********************** The program will NOT restart **********************\n"<< std::endl;
	}
	std::cerr << "***********************************CRASH***********************************\n"<< std::endl;
	usleep(3000000);
}

void atAbort(int) {
	commonAlert("SIGABRT", false);
	writeLog("SIGABRT");
}

void atQuickExit() {
	commonAlert("std::quick_exit()");
	writeLog("std::quick_exit()");
	restartSystem();
}

void atTerminate() {
    commonAlert("std::terminate()");
    writeLog("std::terminate()", true);
    restartSystem();
}

void atSegfault(int) {
	commonAlert("SEGFAULT!");
	writeLog("SEGFAULT!");
	restartSystem();
}

void registerAll() {
	std::at_quick_exit(&atQuickExit);
	std::set_terminate(&atTerminate);
	std::signal(SIGABRT, &atAbort);
	std::signal(SIGSEGV, &atSegfault);
}

void restartSystem() {
	std::exit(RESTART_EXIT_CODE);
}
	
}
}
