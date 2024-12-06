#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include <iostream>

constexpr const char* window_name = "Face Detection";

struct Target {
	int32_t x, y; // relative to frame center
};

static inline Target FindTarget(cv::Mat& frame, cv::CascadeClassifier& cascade, cv::CascadeClassifier& nestedCascade, double scale) {

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

		/*
		cv::Mat smallFrameRoi = smallFrame(face);
		nestedCascade.detectMultiScale(smallFrameRoi, nestedObjects, 1.1, 2, cv::CASCADE_SCALE_IMAGE, 
			cv::Size(30, 30));
		for (cv::Rect& nested : nestedObjects) {
			center.x = cvRound((face.x + nested.x + nested.width * 0.5) * scale);
			center.y = cvRound((face.y + nested.y + nested.height * 0.5) * scale);
			radius = cvRound((nested.width + nested.height) * 0.25 * scale);
			cv::circle(frame, center, radius, color, 3, 8, 0);
		}
		*/
	}

	cv::circle(frame, { frameCenter.x - target.x, frameCenter.y - target.y }, 2, drawColor2, 3, 8, 0);

	cv::imshow(window_name, frame);

	target.y = frame.rows / 2 - (frame.rows - (frameCenter.y - target.y));

	return target;
}

int main() {
	static constexpr double scale = 1.0;
	cv::VideoCapture camCapture;
	cv::Mat camFrame;
	cv::CascadeClassifier cascade, nestedCascade;
	nestedCascade.load("opencv/data/haarcascades/haarcascade_eye.xml");
	cascade.load("opencv/data/haarcascades/haarcascade_frontalface_default.xml");
	if (cascade.empty() || nestedCascade.empty()) {
		std::cout << "failed to open cascade files!" << std::endl;
		return -1;
	}
	camCapture.open(1);
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
		FindTarget(camFrame, cascade, nestedCascade, scale);
		cv::pollKey();
	}
	if (cv::getWindowProperty(window_name, cv::WindowPropertyFlags::WND_PROP_VISIBLE)) {
		cv::destroyWindow("Face");
	}
}
