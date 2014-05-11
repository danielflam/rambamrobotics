#ifndef _ARDUINO_H
#define _ARDUINO_H



#ifdef ARDUINO
#undef ARDUINO
#endif

#define ARDUINO 105

#include "arduino.h"

__extension__ typedef int __guard __attribute__((mode (__DI__)));

extern "C" int __cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release (__guard *);
extern "C" void __cxa_guard_abort (__guard *);


#endif
