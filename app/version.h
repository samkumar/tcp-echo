
#ifdef ROOM_TYPE
#define BUILD_NUMBER 120
#elif defined(DUCT_TYPE)
#define BUILD_NUMBER 125
#else
#error must define DUCT_TYPE or ROOM_TYPE
#endif
