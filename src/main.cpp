#include "wiringPi.h"
#include "softPwm.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <cstdint>
#include <time.h>
#include <math.h>
#include <vector>

struct Vec2 {

	int x{}, y{};

	int SqrMagnitude() const {
		return x * x + x * y;
	}

	Vec2 operator-(Vec2 b) {
		return Vec2 { x - b.x, y - b.y };
	}
};

constexpr Vec2 pic_res { 1280, 720 };
constexpr Vec2 pic_center { 1280 / 2, 720 / 2 };
constexpr int x_motor_pwm = 3;
constexpr int x_motor_0 = 4;
constexpr int x_motor_1 = 5;
constexpr int y_motor_pwm = 0;
constexpr int y_motor_0 = 1;
constexpr int y_motor_1 = 2;

struct Target {
	Vec2 m_PicPos;
};

static inline void GetCameraView(int& outWidth, int& outHeight, std::vector<char[3]>& outPixels) {
}

static inline void GetTargets(size_t* outCount, Target** ppOutTargets) {
	int width, height;
	std::vector<char[3]> pixels;
	GetCameraView(width, height, pixels);
};

static inline int Clamp(int val, int min, int max) {
	val = val > min ? val : min;
	return val < max ? val : max;
}

static inline void RotateMotors(Vec2 relativePos, const Target& target) {
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
		int smallestMagnitude = 1000000;
		Vec2 closestPos = Vec2 { 0, 0 };
		size_t closestIndex = SIZE_MAX;
		for (size_t i = 0; i < targetCount; i++) {
			Vec2 relativePos = targets[i].m_PicPos - pic_center;
			int mag = relativePos.SqrMagnitude();
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
