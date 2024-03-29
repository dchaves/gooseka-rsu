#ifndef GOOSEKA_HELPERS_H
#define GOOSEKA_HELPERS_H

#define ENABLE_DEBUG false // Enable or disable USB Serial console

#if ENABLE_DEBUG // Serial printing enabled
#define DEBUG_BEGIN(BAUDS) Serial.begin(BAUDS)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#else // Serial printing disabled
#define DEBUG_BEGIN(BAUDS) 
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT(...) 
#endif /* ENABLE_DEBUG */

#endif /* GOOSEKA_HELPERS_H */