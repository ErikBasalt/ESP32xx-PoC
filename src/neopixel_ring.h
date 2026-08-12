#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void eraseNeopixelRing(void);
bool startNeopixelRing(void);
void loopNeopixelRing(unsigned long currentMillis);

#ifdef __cplusplus
}
#endif
