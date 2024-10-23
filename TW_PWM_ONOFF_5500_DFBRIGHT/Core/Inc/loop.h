#ifndef INC_LOOP_H_
#define INC_LOOP_H_
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern void setup();
extern bool loop();
extern void runMotors();
extern void assertBootloaderJump();
extern bool isPowerFailed();

#ifdef __cplusplus
}
#endif

#endif /* INC_ADDRLED_H_ */
