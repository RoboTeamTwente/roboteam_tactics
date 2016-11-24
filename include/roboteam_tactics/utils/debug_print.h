#include <cstdio>

#define RTT_xstr(a) RTT_str(a)
#define RTT_str(a) #a
#define RTT_PASTER3(x, y, z) x ## _ ## y ## _ ## z

#define SET_DEBUG_FOR(tag, status) \
static const bool PRINT_ ## tag ## _MESSAGES = status

#define RTT_DEBUG_TAG(tag, format, ...) \
if (RTT_PASTER3(rtt::PRINT, tag, MESSAGES)) { \
    printf("[" RTT_xstr(tag) "] " format, ##__VA_ARGS__); \
}

#define RTT_DEBUG(format, ...) \
RTT_DEBUG_TAG( RTT_CURRENT_DEBUG_TAG , format, ##__VA_ARGS__)

// Enable below here which debug statements should be on and/or off

namespace rtt {

SET_DEBUG_FOR(AimAt, false);
SET_DEBUG_FOR(NaiveBlockGoal, false);
SET_DEBUG_FOR(ParamCheck, false);
SET_DEBUG_FOR(ParamSet, false);

} // rtt
