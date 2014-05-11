/*
 * particleFilter.h
 *
 *  Created on: Apr 7, 2014
 *      Author: Danny
 */

#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

//
//
// IMAGING UNIT FILTER
//
//

#include <Arduino.h>
#include <fastRandom.h>

#define NPARTICLES 20

class Observation {
public:
	Observation() {
	//	ImuHeading = 0;
	//	sonar1 = 0;
		//sonar2 = 0;
	//	ImuSpeed = 0;
	}

	float ImuHeading;
	float ImuSpeed;
	float sonar1;
};

struct Point {
	Point()
	{
		reset();
	}

	Point(float x0, float y0) :
			x(x0), y(y0) {
	}

	Point & operator+(const Point & other)
	{
		x += other.x;
		y += other.y;
		return *this;
	}

	Point & operator/(const float val)
	{
		x /= val;
		y /= val;
		return *this;
	}

	void reset()
	{
		x = 0;
		y = 0;
	}

	float x;
	float y;
};

class Segment {
public:
	Segment * set(float x1, float y1, float x2, float y2) {
		a.x = x1;
		a.y = y1;
		b.x = x1;
		b.y = y1;

		return this;
	}

	Point a;
	Point b;
	// returns a large number if the segment is out of bounds!
	virtual float distance(float x, float y, float angle) = 0;
};

class VerticalSegment: public Segment {
public:
	Point a;
	Point b;

	virtual float distance(float x, float y, float angle) {
		float res = 9999.0;

		float cosAngle = cos(angle);
		if (cosAngle > 0) {
			res = (a.x - x) / cosAngle;
			float a0 = res * sin(angle);
			float dist = y + a0;
			if (dist > b.y || dist < a.y) {
				return 9999.0;
			}

		}

		return abs(res);
	}
};

class HorizontalSegment: public Segment {
public:
	Point a;
	Point b;

	virtual float distance(float x, float y, float angle) {
		float res = 9999.0;

		float sinAngle = sin(angle);
		if (sinAngle > 0) {

			res = (a.y - y) / sinAngle;
			float b0 = res * cos(angle);
			float dist = x + b0;
			if (dist > b.x || dist < a.x) {
				return 9999.0;
			}
		}
		return abs(res);
	}
};

class WorldMap {
public:

	WorldMap() {
		minX = 0;
		minY = 0;
		maxX = 1820; // mm
		maxY = 2440; //mm
		deltaX = maxX - minX;
		deltaY = maxY - minY;

		boundaries[0] = topLine.set(0, 0, 1.820, 0);
		boundaries[1] = bottomLine.set(0, 2.440, 1.820, 2.440);
		boundaries[2] = leftBoundary.set(0, 0, 0, 2.440);
		boundaries[3] = rightBoundary.set(1.820, 0, 1.820, 2.440);
		boundaries[4] = topGoal.set(6.10, 3.00, 6.10 + 6.00, 3.00);
		boundaries[5] = bottomGoal.set(6.10, 2.440 - 3.00, 6.10 + 6.00, 2.440 - 3.00);

	}

	float closestSegment(float x, float y, float heading) {

       float res = 9999.0;

       for (int i = 0; i<6; i++)
       {
    	   float tmp = boundaries[i]->distance(x, y, heading);
    	   if (tmp < res)
    	   {
    		   res = tmp;
    	   }
       }
	   return res;
	}


	float minX, minY, maxX, maxY;
	float deltaX;
	float deltaY;

	Segment * boundaries[6];
	HorizontalSegment topLine;
	HorizontalSegment bottomLine;
	VerticalSegment leftBoundary;
	VerticalSegment rightBoundary;
	HorizontalSegment topGoal;
	HorizontalSegment bottomGoal;

};

class Particle {
public:
	Particle() {
		x = 0.0;
		y = 0.0;
		weight = random();
		//heading = 0.0;
	}

	Particle & operator= (const Particle & other)
	{
		x = other.x;
		y = other.y;
		weight = other.weight;
		//heading = other.heading;

		return *this;
	}

	void randomize(WorldMap *map) {
		x = map->minX + random() * map->deltaX;
		x = map->minY + random() * map->deltaY;
	}

	float update(WorldMap * map, Observation & observation) {
		// this function takes a particle and an updates the likelyhood (between 0.0 and 1.0)


		// move the particle based on the observed speed and direction

		float heading = observation.ImuHeading +  0.2 * fastRandom() - 0.1; // randomize the heading

		// randomize the movement
//		float deg2rad = heading / 180.0;


		x += observation.ImuSpeed * sin(heading) + 10*fastRandom() - 5;
		y += observation.ImuSpeed * cos(heading) + 10*fastRandom() - 5;


		// now lets caluclate the probability of this particle *actually* being at this spot

		// one sonar behind the bot
		float minDistance = map->closestSegment(x, y, heading + PI);

		float hypot = sqrt(minDistance*minDistance + observation.sonar1);

		float res = (1.0-hypot);
		if (res > 1)
			res = 0.9;
		else if (res < 0.1)
			res = 0.1;

		// of this particle being observed based on the observation

		// the probability based on SONAR1~rotation X SONAR2~rotation

		// the probability based on line crossing??

		return res;
	}


	float x, y;
	float weight; // probability of being in said spot copared to all other particles
};

class ParticleFilter {

public:
	Particle particles[NPARTICLES];
	Particle particles2[NPARTICLES];

	float estimatedX, estimatedY;
	WorldMap * worldmap;

	float bestX, bestY;
public:

	ParticleFilter(WorldMap * map) {
		fastRandomSeed(random());
		worldmap = map;
		initParticles();
		bestX = 0.0;
		bestY = 0.0;
	}

	void update(Observation & observation) {
		// update the position and the sensor probabilities
		for (int i = 0; i < NPARTICLES; i++) {
			particles[i].update(worldmap, observation);
		}

		resample();

		// normalize the weights

	}

	void getEstimatedLocation(Point & loc /* out */)
	{
		loc.reset();

		for (int i = 0; i < NPARTICLES; i++) {
			loc.x += particles[i].x;
			loc.y += particles[i].y;
		}
		loc.x /= float(NPARTICLES);
		loc.y /= float(NPARTICLES);
	}

private:
	void initParticles() {
		for (int i = 0; i < NPARTICLES; i++) {
			particles[i].randomize(worldmap);
		}
	}

	void resample() {
		float sumW = 0;
		for (int i = 0; i < NPARTICLES; i++) {
			sumW += particles[i].weight;
		}

		int index = 0;
		float step = sumW / float(NPARTICLES);
		float beta = step;

		for (int i = 0; i < NPARTICLES;) {
			while (beta > particles[i].weight) {
				beta -= particles[i].weight;
				index = (index + 1) % NPARTICLES;
			}
			beta += fastRandom() * step;
			if (fastRandom() > 0.5)
				particles2[i++] = particles[index];
		}

		for (int i = 0; i < NPARTICLES;) {
			particles[index] = particles2[index];
		}

		// randomize a particle every so often;

		int i = int((fastRandom() * 100 * NPARTICLES));
		if (i > 0 && i < NPARTICLES)
			particles[i].randomize(worldmap);


		/*
		 index = 0
		 totw = sum(w)
		 incr = totw / N
		 beta = incr
		 while len(p3) < N:
		 while beta > w[index]:
		 beta -= w[index]
		 index = (index + 1) % N
		 beta += random.random() * incr
		 if random.random() > 0.5:
		 p3.append(p[index])   */
	}

};

#endif /* PARTICLEFILTER_H_ */
