// https://github.com/canadaduane/LEDSensor

#ifndef LEDSensor_h
#define LEDSensor_h

class LEDSensor {

public:
	LEDSensor(byte ledPin = 0) :
			mLedPin(0), mMeasureAnalog(false), mAnalogThresholdLevel(
					0), mMaxDelta(2000), mStartTime(-1), mTimeDelta(0), nmeasured(
					0), cumulativeTime(0), next(0) {
		attach(ledPin);
	}

	void attach(int ledPin);
	void setAnalogMeasurement(int thresholdLevel); // measure from an analog pin
	void setDigitalMeasurement(); // measure from a digital pin (default)

	inline long getValue() {
		return mTimeDelta;
	}
	inline double getAverage() {
		return nmeasured > 0 ? double(cumulativeTime) / double(nmeasured) : 0.0;
	}

	inline void resetAverage() {
		cumulativeTime = 0;
		nmeasured = 0;
	}

	static void refresh();
	void measure();

protected:
	uint8_t mLedPin;
	bool mMeasureAnalog;
	int mAnalogThresholdLevel; // (0 to 1023)

	long mMaxDelta;
	long mStartTime;
	long mTimeDelta;
	int nmeasured;
	unsigned long cumulativeTime;

	void charge();
	void discharge();

	bool isDischarged();
	bool isTimedOut();

private:
	class LEDSensor* next;
	static LEDSensor* first;

};

#endif //LEDSensor_h
