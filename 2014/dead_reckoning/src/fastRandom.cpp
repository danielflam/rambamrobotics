/*
 * fastRandom.cpp
 *
 *  Created on: Apr 7, 2014
 *      Author: Danny
 */

#define RAND_A 16807
#define RAND_M 2147483647
#define RAND_Q 127773
#define RAND_R 2836
#define RAND_SCALE (1.0 / RAND_M)

#include <stdint.h>
#include <stdlib.h>

#include <fastRandom.h>

int32_t seed;

void fastRandomSeed(int32_t a_seed)
{
	seed = a_seed;
}

float fastRandom() {
	int32_t k = seed / RAND_Q;
	seed = RAND_A * (seed - k * RAND_Q) - k * RAND_R;
	if (seed < 0)
		seed += RAND_M;
	return float(seed * (float)RAND_SCALE);
}

