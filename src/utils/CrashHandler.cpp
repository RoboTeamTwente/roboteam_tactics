#include "roboteam_tactics/utils/CrashHandler.h"

#ifdef CURSES_HAVE_NCURSES_H
#include <ncurses.h>
#elif CURSES_HAVE_NCURSES_NCURSES_H
#include <ncurses/ncurses.h>
#else
//#warning "ncurses was not found, you won't see an error message if the program crashes and rtt::crash::registerAll has been called"
//#define NO_NCURSES
#include <ncurses.h>
#endif

#include <unistd.h>

namespace rtt {
namespace crash {

void commonAlert(std::string source, bool restarting) {
#ifndef NO_NCURSES
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
#endif
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

void registerAll() {
	std::at_quick_exit(&atQuickExit);
	std::set_terminate(&atTerminate);
	std::signal(SIGABRT, &atAbort);
}

void restartSystem() {
	std::exit(RESTART_EXIT_CODE);
}
	
}
}