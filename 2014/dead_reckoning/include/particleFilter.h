/*
 * particleFilter.h
 *
 *  Created on: Apr 7, 2014
 *      Author: Danny
 */

#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include <Arduino.h>
#include <fastRandom.h>
#include <math.h>

#define NPARTICLES 10
#define GYRO_TO_RAD 0.001214142f //(14.375^-1 * pi/180)
#define GYRO_TO_DEG 0.069565217f

#define ADJD_S311_WHITE_RED 179
#define ADJD_S311_WHITE_GREEN 178
#define ADJD_S311_WHITE_BLUE 165
#define ADJD_S311_WHITE_TOT 522
#define ADJD_S311_WHITE_CLEAR 589


struct Vector3D {
//	Vector3D & operator-(Vector3D & other)
//	{
//		x = other.x;
//		y = other.y;
//		z = other.z;
//
//		return *this;
//	}

	Vector3D() {
		x = 0;
		y = 0;
		z = 0;
	}

	Vector3D & operator=(Vector3D & other) {
		x = other.x;
		y = other.y;
		z = other.z;

		return *this;
	}

	Vector3D & operator-=(Vector3D & other) {
		x -= other.x;
		y -= other.y;
		z -= other.z;

		return *this;
	}

	Vector3D & operator+=(Vector3D & other) {
		x += other.x;
		y += other.y;
		z += other.z;

		return *this;
	}

	Vector3D & operator>>=(int i) {
		x >>= i;
		y >>= i;
		z >>= i;

		return *this;
	}

	void log(const char * s = NULL) {
		if (s) {
			Serial.print(s);
		}

		Serial.print(x);
		Serial.print(" ");
		Serial.print(y);
		Serial.print(" ");
		Serial.println(z);
	}

	int muldiv(int i, int j, int k)
	{
		long tmp = long(i) * long(j);
		return int(tmp / k);
	}

	void muldiv(int i, int k)
	{
		x = muldiv(x, i, k);
		y = muldiv(y, i, k);
		z = muldiv(z, i, k);
	}
	int16_t x;
	int16_t y;
	int16_t z;
};

class Observation {
public:
	Observation() {
		//	ImuHeading = 0;
		sonar1 = 0;
		//sonar2 = 0;
		//	ImuSpeed = 0;

	}

	int muldivcolor(Vector3D color, int Rmul, int Gmul, int Bmul)
	{
		long tmp = long(color.x) * long(Rmul) + long(color.y) * long(Gmul) + long(color.z) * long(Bmul);

		return int(tmp / 1000);
	}
	void processIMUReading() {
//		accelerometerRaw -= accelerometerCalibration;
//		gyroRaw -= gyroCalibration;
//		magnetometerRaw -= magnetometerCalibration; // startup calib is "north"

		headingMagnetic = atan2(float(magnetometerRaw.y),
				float(magnetometerRaw.x));

		// normalize colors to same level intensity as reference
		color.muldiv(  ADJD_S311_WHITE_CLEAR, colorClear);

		yuv.x = muldivcolor(color, 299, 587, 114);
		yuv.y = muldivcolor(color, -169, -332, 500);
		yuv.z = muldivcolor(color, 500, -419, 81);
		colorNotGreen = (4*color.x + 6*color.z)/10;


	}

	void log() {
		accelerometerRaw.log("acc: ");
		gyroRaw.log("gyro: ");
		magnetometerRaw.log("mag: ");
		color.log("color: ");
		yuv.log("yuv: ");
		Serial.print(" initial:");

		Serial.print(" clear:");
		Serial.print(colorClear);
		Serial.print(" notGreen:");
		Serial.print(colorNotGreen);

		Serial.print(" heading:");
		Serial.print(initialHeadingMagnetic),
		Serial.print(" current:");
		Serial.print(headingMagnetic);
		Serial.print(" sonar:");
		Serial.println(sonar1);
	}

	Vector3D accelerometerRaw;
	Vector3D gyroRaw;
	Vector3D magnetometerRaw;
	Vector3D color; // from adjd_s311
	Vector3D yuv;
	int16_t colorClear;
	int16_t colorNotGreen;


//	Vector3D accelerometerCalibration;
//	Vector3D gyroCalibration;
//	Vector3D magnetometerCalibration;

	float headingMagnetic;
	float initialHeadingMagnetic;
	float sonar1;

};

struct Point {
	Point() {
		reset();
	}

	Point(float x0, float y0) :
			x(x0), y(y0) {
	}

	Point & operator+(const Point & other) {
		x += other.x;
		y += other.y;
		return *this;
	}

	Point & operator/(const float val) {
		x /= val;
		y /= val;
		return *this;
	}

	void reset() {
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
		b.x = x2;
		b.y = y2;

		return this;
	}

	Point a;
	Point b;
	// returns a large number if the segment is out of bounds!

//	// Copyright 2001 softSurfer, 2012 Dan Sunday
//	// This code may be freely used and modified for any purpose
//	// providing that this copyright notice is included with it.
//	// SoftSurfer makes no warranty for this code, and cannot be held
//	// liable for any real or imagined damage resulting from its use.
//	// Users of this code must verify correctness for their application.
//
//
//	// Assume that classes are already given for the objects:
//	//     Point and Vector with
//	//          coordinates {float x, y, z;} (z=0  for 2D)
//	//          appropriate operators for:
//	//               Point  = Point ± Vector
//	//               Vector = Point - Point
//	//               Vector = Scalar * Vector
//	//     Line with defining endpoints {Point P0, P1;}
//	//     Segment with defining endpoints {Point P0, P1;}
//	//===================================================================
//
//	// dot product (3D) which allows vector operations in arguments
//	#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
//	#define norm(v)     sqrt(dot(v,v))     // norm = length of  vector
//	#define d(u,v)      norm(u-v)          // distance = norm of difference
//
//
//
//	// closest2D_Point_to_Line(): find the closest 2D Point to a Line
//	//     Input:  an array P[] of n points, and a Line L
//	//     Return: the index i of the Point P[i] closest to L
//	int
//	closest2D_Point_to_Line( Point P[], int n, Line L)
//	{
//	     // Get coefficients of the implicit line equation.
//	     // Do NOT normalize since scaling by a constant
//	     // is irrelevant for just comparing distances.
//	     float a = L.P0.y - L.P1.y;
//	     float b = L.P1.x - L.P0.x;
//	     float c = L.P0.x * L.P1.y - L.P1.x * L.P0.y;
//
//	     // initialize min index and distance to P[0]
//	     int mi = 0;
//	     float min = a * P[0].x + b * P[0].y + c;
//	     if (min < 0) min = -min;     // absolute value
//
//	     // loop through Point array testing for min distance to L
//	     for (i=1; i<n; i++) {
//	          // just use dist squared (sqrt not  needed for comparison)
//	          float dist = a * P[i].x + b * P[i].y  + c;
//	          if (dist < 0) dist = -dist;    // absolute value
//	          if (dist < min) {      // this point is closer
//	               mi = i;              // so have a new minimum
//	               min = dist;
//	          }
//	     }
//	     return mi;     // the index of the closest  Point P[mi]
//	}
//	//===================================================================
//
//
//	// dist_Point_to_Line(): get the distance of a point to a line
//	//     Input:  a Point P and a Line L (in any dimension)
//	//     Return: the shortest distance from P to L
//	float
//	dist_Point_to_Line( Point P, Line L)
//	{
//	     Vector v = L.P1 - L.P0;
//	     Vector w = P - L.P0;
//
//	     double c1 = dot(w,v);
//	     double c2 = dot(v,v);
//	     double b = c1 / c2;
//
//	     Point Pb = L.P0 + b * v;
//	     return d(P, Pb);
//	}
//	//===================================================================
//
//
	virtual float distance(float x, float y, float angle) = 0;


//#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y
//	#define norm(v)     sqrt(dot(v,v))     // norm = length of  vector
//	#define d(u,v)      norm(u-v)          // distance = norm of difference
	//	// dist_Point_to_Segment(): get the distance of a point to a segment
	//	//     Input:  a Point P and a Segment S (in any dimension)
	//	//     Return: the shortest distance from P to S
	//	float
	//	dist_Point_to_Segment( Point P, Segment S)
	//	{
	//	     Vector v = S.P1 - S.P0;
	//	     Vector w = P - S.P0;
	//
	//	     double c1 = dot(w,v);
	//	     if ( c1 <= 0 )
	//	          return d(P, S.P0);
	//
	//	     double c2 = dot(v,v);
	//	     if ( c2 <= c1 )
	//	          return d(P, S.P1);
	//
	//	     double b = c1 / c2;
	//	     Point Pb = S.P0 + b * v;
	//	     return d(P, Pb);
	//	}

	float rayDistance(float x, float y)
	{
		float vx = b.x - a.x;
		float vy = b.y - a.y;
		float wx = x - a.x;
		float wy = x - a.y;
		float c1 = wx*vx + wy*vy;
		if (c1 <= 0)
		{
			return sqrt( wx*wx + wy*wy );
		}

		float c2 = vx*vx + vy*vy;
		if (c2 <= c1 )
		{
			wx = x - b.x;
			wy = x - b.y;
			return sqrt( wx*wx + wy*wy );
		}
		float b = c1/c2;
		vx = b*vx + a.x;
		vy = b*vy + a.y;
		wx = vx-x;
		wy = vy-y;

		return sqrt( wx*wx + wy*wy );
	}
};

class VerticalSegment: public Segment {
public:

	virtual float distance(float x, float y, float angle) {
		float res = 9999.0;
//		Serial.print("horz ");
//		Serial.print(x);
//		Serial.print(" ");
//		Serial.print(y);
//		Serial.print(" ");
//		Serial.print(angle);
//		Serial.print(" ");
//		Serial.print(a.x);
//		Serial.print(" ");
//		Serial.print(a.y);
//		Serial.print(" ");
//		Serial.print(b.x);
//		Serial.print(" ");
//		Serial.print(b.y);

		float cosAngle = cos(angle);
		if (cosAngle != 0) {
			res = (a.x - x) / cosAngle;
			float a0 = res * sin(angle);
			float dist = y + a0;

//			Serial.print(" ");
//			Serial.print(res);
//			Serial.print(" ");
//			Serial.print(a0);
//			Serial.print(" ");
//			Serial.print(dist);

			if (dist > b.y || dist < a.y) {
				res = 9999.0;
			}
//			else
//				Serial.print("******");

		}

		//Serial.println();

		return abs(res);
	}
};


class HorizontalSegment: public Segment {
public:


	virtual float distance(float x, float y, float angle) {
		float res = 9999.0;

//		Serial.print("vert ");
//		Serial.print(x);
//		Serial.print(" ");
//		Serial.print(y);
//		Serial.print(" ");
//		Serial.print(angle);
//		Serial.print(" ");
//		Serial.print(a.x);
//		Serial.print(" ");
//		Serial.print(a.y);
//		Serial.print(" ");
//		Serial.print(b.x);
//		Serial.print(" ");
//		Serial.println(b.y);

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

// in a circle "a" represents the center and "b.x" and "b.y" the radius

class CircleSegment: public Segment {
public:

	virtual float distance(float x, float y, float angle) {
		float deltax =  (a.x - x);
		float deltay = (a.y - y);
		return abs(sqrt(deltax*deltax + deltay+deltay) - b.x);
	}
};

class WorldMap {
public:

	WorldMap() {
		minX = 0;
		minY = 0;
		maxX = 182.0; // cm
		maxY = 244.0; //cm
		deltaX = maxX - minX;
		deltaY = maxY - minY;

		boundaries[0] = topLine.set(0, 0, 182.0, 0);
		boundaries[1] = bottomLine.set(0, 244.0, 182.0, 244.0);
		boundaries[2] = leftBoundary.set(0, 0, 0, 244.0);
		boundaries[3] = rightBoundary.set(182.0, 0, 182.0, 244.0);
		boundaries[4] = topGoal.set(61.0, 30.0, 61.0 + 60.0, 30.0);
		boundaries[5] = bottomGoal.set(61.0, 244.0 - 30.0, 61.0 + 60.0,
				244.0 - 30.0);

		// 30, 30 , 152, 214
		lines[0] = topWhiteLine.set(30,30,152,30);
		lines[1] = bottomWhiteLine.set(30,214,152,214);
		lines[2] = leftWhiteLine.set(30,30,30,214);
		lines[3] = rightWhiteLine.set(152,30,152,214);

		// black lines
		lines[4] = bottomGoalSegment1.set(46,30,46,60);
		lines[5] = bottomGoalSegment2.set(46,60,136,60);
		lines[6] = bottomGoalSegment3.set(136,30,136,60);

		lines[7] = topGoalSegment1.set(46,184,46,244);
		lines[8] = topGoalSegment2.set(46,184,136,184);
		lines[9] = topGoalSegment3.set(136,184,136,244);

		lines[10] = gameMidCircle.set(91,122,30,30);
	}


	float closestBoundaries(float x, float y, float heading) {

		float res = 9999.0;

		for (int i = 0; i < 6; i++) {
			float tmp = boundaries[i]->distance(x, y, heading);
			if (tmp < res) {
				res = tmp;
			}
		}

		return res;
	}

	float closestLine(float x, float y) {
		float res = 9999.0;

		for (int i = 0; i < 6; i++) {
			float tmp = lines[i]->rayDistance(x, y);
			if (tmp < res) {
				res = tmp;
			}
		}

		return res;
	}



	float minX, minY, maxX, maxY;
	float deltaX;
	float deltaY;

	Segment * boundaries[6];
	Segment * lines[11];
	HorizontalSegment topLine;
	HorizontalSegment bottomLine;
	VerticalSegment leftBoundary;
	VerticalSegment rightBoundary;
	HorizontalSegment topGoal;
	HorizontalSegment bottomGoal;

	// white lines
	HorizontalSegment topWhiteLine;
	HorizontalSegment bottomWhiteLine;
	VerticalSegment leftWhiteLine;
	VerticalSegment rightWhiteLine;

	// black lines
	VerticalSegment bottomGoalSegment1;
	HorizontalSegment bottomGoalSegment2;
	VerticalSegment bottomGoalSegment3;

	VerticalSegment topGoalSegment1;
	HorizontalSegment topGoalSegment2;
	VerticalSegment topGoalSegment3;

	//HorizontalSegment gameMidline;
	CircleSegment gameMidCircle;
};

struct Particle {
public:
	Particle() {
		x = 0.0;
		y = 0.0;
		weight = uint8_t(254 * fastRandom());
		speedX = int8_t(gausianish() * 10);
		speedY = int8_t(gausianish() * 10);
		//heading = 0.0;
	}

	Particle & operator=(const Particle & other) {
		x = other.x;
		y = other.y;
		weight = other.weight;
		//heading = other.heading;

		return *this;
	}

	void log() {
		Serial.print(">> p(");
		Serial.print(x);
		Serial.print(",");
		Serial.print(y);
		Serial.print(") w(");
		Serial.print(weight);
		Serial.print(") s(");
		Serial.print(speedX);
		Serial.print(",");
		Serial.print(speedY);
		Serial.println(")");
	}

	void randomize(WorldMap *worldmap) {
		x = map(random(), 0, 1, 0, worldmap->maxX); // map->minX + random() * map->deltaX;
		y = map(random(), 0, 1, 0, worldmap->maxY); //map->minY + random() * map->deltaY;
		weight = uint8_t(fastRandom() * 254);
		speedX = int8_t(gausianish() * 10);
		speedY = int8_t(gausianish() * 10);
	}

	float gausianish() {

		return (fastRandom() + fastRandom()) - 1.0;
	}

	float update(WorldMap * worldmap, Observation & observation) {
		// this function takes a particle and an updates the likelyhood (between 0.0 and 1.0)

		// move the particle based on the observed speed, direction and gyro

		// remove offset - 0 is our original direction!
		float heading = observation.headingMagnetic - observation.initialHeadingMagnetic;

		// add some noise fun - from gyro
		heading += GYRO_TO_RAD * float(observation.gyroRaw.z) / 500 + 0.1 * gausianish();

		// our new speed is sort of...
		float vx = float(speedX) + (float(observation.accelerometerRaw.x) / 10.0)
				+ 3*gausianish();
		float vy = float(speedY) + (float(observation.accelerometerRaw.y) / 10.0)
				+ 3*gausianish();


		float floatx = float(x) + vx;
		float floaty = float(y) + vy; // to mm

		//Serial.println(vx);

		speedX = int8_t(vx);
		speedY = int8_t(vy);

		x = constrain(floatx, 0, 255);
		y = constrain(floaty, 0, 255);

		if (x <= worldmap->minX || y <= worldmap->minY|| x >= worldmap->maxX || y >= worldmap->maxY)
		{
			speedX = 0;
			speedY = 0;
		}

		// now lets caluclate the probability of this particle *actually* being at this spot

		float res = 1.0;
		if (observation.colorNotGreen < 200)
		{
		// we are on some type of line
			float minDistance = constrain(worldmap->closestLine(x, y), 10, 244);

			res = constrain(map(minDistance, 0, 244, 0.9, 0.1 ), 0.1, 0.9);
		}

		if (observation.sonar1 > -1) {
			float minDistance = worldmap->closestBoundaries(x, y, heading + PI);

//			Serial.print(minDistance);
//			Serial.print(" ");

			float delta = minDistance - observation.sonar1;
			float prob = constrain( map(delta * delta, 0, 60000, 0.95,0.05), 0.1, 0.9);

			//Serial.println(hypot);

			res *= prob;
		} else {
			//float delta = atan2(vx, vy) - heading;
			res = 0.5;
		}

		// fuzzify it more....
		if (res > 0.9)
			res = 0.9;
		else if (res < 0.1)
			res = 0.1;

		// of this particle being observed based on the observation

		// the probability based on SONAR1~rotation X SONAR2~rotation

		// the probability based on line crossing??

		weight = uint8_t(res * 255);

		return res;
	}

	uint8_t x, y;
	uint8_t weight;
	int8_t speedX, speedY;

	//float weight; // probability of being in said spot copared to all other particles
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

	Point & getEstimatedLocation() {
		currentLocationSortOf.reset();

		for (int i = 0; i < NPARTICLES; i++) {
			currentLocationSortOf.x += particles[i].x;
			currentLocationSortOf.y += particles[i].y;
		}
		currentLocationSortOf.x /= float(NPARTICLES);
		currentLocationSortOf.y /= float(NPARTICLES);
		return currentLocationSortOf;
	}

	Point currentLocationSortOf;

	void log() {
		Serial.println("**** particles ****");
		for (int i = 0; i < NPARTICLES; i++)
			particles[i].log();

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
			while (particles[i].weight > 0 && beta > particles[i].weight) {

				beta -= particles[i].weight;
				index = (index + 1) % NPARTICLES;
			}
			beta += fastRandom() * step;

			if (fastRandom() > 0.5) {

				particles2[i++] = particles[index];
			}
		}

		for (int i = 0; i < NPARTICLES; i++) {
			particles[i] = particles2[i];
		}

		// randomize a particle every so often;
		int i = int((fastRandom() * 400 * NPARTICLES));
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
