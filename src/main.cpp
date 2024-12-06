#include "wiringPi.h"
#include "softPwm.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <time.h>
#include <math.h>
#include <vector>
#include <iostream>

constexpr const char* window_name = "Camera View";

constexpr int x_motor_pwm = 3;
constexpr int x_motor_0 = 4;
constexpr int x_motor_1 = 5;
constexpr int y_motor_pwm = 0;
constexpr int y_motor_0 = 1;
constexpr int y_motor_1 = 2;

struct Target {
	int x, y; // relative to frame center
};

static inline Target FindTarget(cv::Mat& frame, cv::CascadeClassifier& cascade, double scale) {

	static const cv::Scalar drawColor1 = cv::Scalar(255, 0, 0);
	static const cv::Scalar drawColor2 = cv::Scalar(0, 0, 255);

	Target target { INT32_MAX, INT32_MAX };
	cv::Point frameCenter = { frame.cols / 2, frame.rows / 2 };

	std::vector<cv::Rect> faces;
	cv::Mat grayFrame, smallFrame;

	cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
	double fx = 1 / scale;
	cv::resize(grayFrame, smallFrame, cv::Size(), fx, fx, cv::INTER_LINEAR);
	cv::equalizeHist(smallFrame, smallFrame);

	cascade.detectMultiScale(smallFrame, faces, 1.1, 2, cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

	if (!faces.size()) {
		return target;
	}

	//std::cout << "detecting faces" << std::endl;

	int32_t closestSqrMag = INT32_MAX;

	for (cv::Rect& face : faces) {

		std::vector<cv::Rect> nestedObjects;
		cv::rectangle(frame, face, drawColor1, 3, 8, 0);

		cv::Point faceCenter;
		faceCenter.x = cvRound((face.x + face.width * 0.5) * scale);
		faceCenter.y = cvRound((face.y + face.height * 0.5) * scale);

		Target newTarget = { frameCenter.x - faceCenter.x, frameCenter.y - faceCenter.y };
		int32_t sqrMag = target.x * target.x + target.y * target.y;

		if (sqrMag < closestSqrMag) {
			closestSqrMag = sqrMag;
			target = newTarget;
		}
	}

	cv::circle(frame, { frameCenter.x - target.x, frameCenter.y - target.y }, 2, drawColor2, 3, 8, 0);

	cv::imshow(window_name, frame);

	target.y = frame.rows / 2 - (frame.rows - (frameCenter.y - target.y));

	return target;
}

static inline int Clamp(int val, int min, int max) {
	val = val > min ? val : min;
	return val < max ? val : max;
}

static inline void RotateMotors(cv::Point picCenter, Target target) {
	if (target.x == INT32_MAX || target.y == INT32_MAX) {
		return;
	}
	int xSpeed = (int)Clamp(abs(target.x) / picCenter.x * 100, 0.0, 100.0);
	int ySpeed = (int)Clamp(abs(target.y) / picCenter.y * 100, 0.0, 100.0);
	if (target.x > 0) {
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
	if (target.y > 0) {
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

	cv::VideoCapture camCapture;
	cv::Mat camFrame;
	cv::CascadeClassifier cascade;
	cascade.load("opencv/data/haarcascades/haarcascade_frontalface_default.xml");

	if (cascade.empty()) {
		std::cout << "failed to open face cascade file!" << std::endl;
		return -1;
	}

	camCapture.open(0);

	if (!camCapture.isOpened()) {
		std::cout << "failed to start camera capture!" << std::endl;
		return -1;
	}

	cv::namedWindow(window_name);


	while (camCapture.isOpened() && cv::getWindowProperty(window_name, cv::WindowPropertyFlags::WND_PROP_VISIBLE)) {
		camCapture >> camFrame;
		if (camFrame.empty()) {
			std::cout << "camera frame was empty!" << std::endl;
			break;
		}
		int resolution[2] = { camFrame.rows, camFrame.cols };
		RotateMotors({ camFrame.cols / 2, camFrame.rows / 2 }, FindTarget(camFrame, cascade, 1.0));
		cv::pollKey();
	}

	if (cv::getWindowProperty(window_name, cv::WindowPropertyFlags::WND_PROP_VISIBLE)) {
		cv::destroyWindow("Face");
	}

	softPwmWrite(x_motor_pwm, 0);
	softPwmWrite(y_motor_pwm, 0);

	digitalWrite(y_motor_0, LOW);
	digitalWrite(y_motor_1, LOW);
	digitalWrite(x_motor_0, LOW);
	digitalWrite(x_motor_1, LOW);

	return 0;
}
