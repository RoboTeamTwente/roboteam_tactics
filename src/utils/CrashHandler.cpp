#include "roboteam_tactics/utils/CrashHandler.h"
#include <ncurses.h>
#include <unistd.h>

namespace rtt {
namespace crash {

void commonAlert(std::string source, bool restarting) {
	initscr();
	start_color();	
	init_pair(1, COLOR_RED, COLOR_BLACK);
	attron(COLOR_PAIR(1));
	printw("\n\n\n\n\n\n\n\n\n\n");
	printw("***********************************CRASH***********************************\n");
	printw("*************************** The program crashed! **************************\n");
	printw("******************** Cause: ");
	printw(source.c_str());
	printw("  ****************************\n");
	if (restarting) {
		printw("*********************** The program will now restart **********************\n");
	} else {
		printw("*********************** The program will NOT restart **********************\n");
	}
	printw("***********************************CRASH***********************************\n");
	attroff(COLOR_PAIR(1));
	refresh();
	flash();
	usleep(3000000);
	flash();
	refresh();
	endwin();
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