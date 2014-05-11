#include "arduino_app.h"
#include <pins_arduino.h>

#include "Servo/Servo.h"
#include "soccerMotors.h"
#include "soccerPID.h"



void * operator new(size_t size)
{
  return malloc(size);
}

void operator delete(void * ptr)
{
  free(ptr);
}

void * operator new[](size_t size)
{
    return malloc(size);
}

void operator delete[](void * ptr)
{
	if (ptr)
		free(ptr);
}


extern "C" void __cxa_pure_virtual()
{
  cli();
  for (;;);
}


extern "C" {
	int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
	void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
	void __cxa_guard_abort (__guard *) {};
}


//
//int main(void) {
//	init();
//
//	setup();
//
//	for (;;)
//		loop();
//
//	return 0;
//}

