#include "roboteam_tactics/utils/CrashHandler.h"
//#include <ncurses.h>
#include <unistd.h>
#include <iostream>

namespace rtt {
namespace crash {

void commonAlert(std::string source, bool restarting) {
	//initscr();
	//start_color();	
	//init_pair(1, COLOR_RED, COLOR_BLACK);
	//attron(COLOR_PAIR(1));
	
	std::cout << "\n\n\n\n\n\n\n\n\n\n" << std::endl;
	std::cout << "***********************************CRASH***********************************\n" << std::endl;
	std::cout << "*************************** The program crashed! **************************\n" << std::endl;
	std::cout << "******************** Cause: " << std::endl;
	std::cout << source.c_str()<< std::endl;
	std::cout << "  ****************************\n"<< std::endl;
	if (restarting) {
		std::cout << "*********************** The program will now restart **********************\n"<< std::endl;
	} else {
		std::cout << "*********************** The program will NOT restart **********************\n"<< std::endl;
	}
	std::cout << "***********************************CRASH***********************************\n"<< std::endl;
	//attroff(COLOR_PAIR(1));
	//refresh();
	//flash();
	usleep(3000000);
	//flash();
	//refresh();
	//endwin();
	if (restarting) {
		restartSystem();
	}
}

void atAbort(int) {
	commonAlert("SIGABT", false); // Don't restart, it's probably a CTRL+C
}

void atQuickExit() {
	commonAlert("std::quick_exit()");
}

void atTerminate() {
    commonAlert("std::terminate()");
}

void atSegfault(int) {
	commonAlert("SEGFAULT!");
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