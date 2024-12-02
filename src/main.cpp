#include "wiringPi.h"
#include "softPwm.h"
#include <cstdint>
#include <time.h>
#include <math.h>

struct Vec2 {

	double x{}, y{};

	inline double SqrMagnitude() {
		return x * x + x * y;
	}

	inline Vec2 operator-(Vec2 b) {
		return Vec2 { x - b.x, y - b.y };
	}
};

constexpr Vec2 pic_res { 1280.0, 720.0 };
constexpr Vec2 pic_center { 1280.0 / 2, 720.0 / 2 };
constexpr int x_motor_pwm = 3;
constexpr int x_motor_0 = 4;
constexpr int x_motor_1 = 5;
constexpr int y_motor_pwm = 0;
constexpr int y_motor_0 = 1;
constexpr int y_motor_1 = 2;

struct Target {
	Vec2 picPos;
};

inline double Clamp(double val, double min, double max) {
	val = val > min ? val : min;
	return val < max ? val : max;
}

inline void GetTargets(size_t* outCount, Target** ppOutTargets) {
};

inline void RotateMotors(Vec2 relativePos, const Target& target) {
	int xSpeed = (int)Clamp(abs(relativePos.x) / pic_center.x * 100, 0.0, 100.0);
	int ySpeed = (int)Clamp(abs(relativePos.y) / pic_center.y * 100, 0.0, 100.0);
	if (relativePos.x > 0) {
		//move right
		pinMode(x_motor_0, HIGH);
		pinMode(x_motor_1, LOW);
		softPwmWrite(x_motor_pwm, xSpeed);
	}
	else {
		//move left
		pinMode(x_motor_0, LOW);
		pinMode(x_motor_1, HIGH);
		softPwmWrite(x_motor_pwm, xSpeed);
	}
	if (relativePos.y > 0) {
		//move up
		pinMode(y_motor_0, HIGH);
		pinMode(y_motor_1, LOW);
		softPwmWrite(y_motor_pwm, ySpeed);
	}
	else {
		//mode down
		pinMode(y_motor_0, LOW);
		pinMode(y_motor_1, HIGH);
		softPwmWrite(y_motor_pwm, ySpeed);
	}
}

int main() {
	wiringPiSetupGpio();
	wiringPiSetup();
	pinMode(x_motor_0, OUTPUT);
	pinMode(x_motor_1, OUTPUT);
	softPwmCreate(x_motor_pwm, 0, 100);
	pinMode(y_motor_0, OUTPUT);
	pinMode(y_motor_1, OUTPUT);
	softPwmCreate(y_motor_pwm, 0, 100);
	digitalWrite(y_motor_0, LOW);
	digitalWrite(y_motor_1, LOW);
	digitalWrite(x_motor_0, LOW);
	digitalWrite(x_motor_1, LOW);
	while (true) {
		size_t targetCount;
		Target* targets;
		GetTargets(&targetCount, &targets);
		double smallestMagnitude = 1000000.0;
		Vec2 closestPos = Vec2 { 0, 0 };
		size_t closestIndex = SIZE_MAX;
		for (size_t i = 0; i < targetCount; i++) {
			Vec2 relativePos = targets[i].picPos - pic_center;
			double mag = relativePos.SqrMagnitude();
			if (mag < smallestMagnitude) {
				smallestMagnitude = mag;
				closestPos = relativePos;
				closestIndex = i;
			}
		}
		if (closestIndex != SIZE_MAX) {
			RotateMotors(closestPos, targets[closestIndex]);
		}
	}
	softPwmWrite(x_motor_pwm, 0);
	softPwmWrite(y_motor_pwm, 0);
	return 0;
}
