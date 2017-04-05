#include <string>
#include <iostream>
#include <ros/ros.h>

namespace {

    std::string const RED_BOLD_COLOR = "\e[1;31m";
    std::string const YELLOW_BOLD_COLOR = "\e[1;33m";
    std::string const BLUE_BOLD_COLOR = "\e[1;34m";
    std::string const END_COLOR = "\e[0m";

} // anonymous namespace

namespace rtt {

void setStdoutToTeam() {
    std::string our_color = "yellow";
    ros::param::getCached("our_color", our_color);

    // @Hack: We use printf here because RTT_DEBUG also uses printf
    // and flushing for this is just too much
    if (our_color == "yellow") {
        printf("%s", YELLOW_BOLD_COLOR.c_str());
    } else {
        printf("%s", BLUE_BOLD_COLOR.c_str());
    }
}

void resetStdoutColor() {
    printf("%s", END_COLOR.c_str());
}

} // namespace rtt

