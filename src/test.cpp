#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include <iostream>

constexpr const char* window_name = "Face Detection";

static inline void DetectAndDraw(cv::Mat& frame, cv::CascadeClassifier& cascade, cv::CascadeClassifier& nestedCascade, double scale) {

	std::vector<cv::Rect> faces, faces2;
	cv::Mat grayFrame, smallFrame;
	cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
	double fx = 1 / scale;
	cv::resize(grayFrame, smallFrame, cv::Size(), fx, fx, cv::INTER_LINEAR);
	cv::equalizeHist(smallFrame, smallFrame);

	cascade.detectMultiScale(smallFrame, faces, 1.1, 2, cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

	if (faces.size()) {
		std::cout << "detecting faces";
	}

	for (cv::Rect& face : faces) {
		cv::Mat smallFrameRoi;
		std::vector<cv::Rect> nestedObjects;
		cv::Point center;
		cv::Scalar color = cv::Scalar(255, 0, 0);
		int radius;
		double aspectRatio = (double)face.width / face.height;
		if (0.75 < aspectRatio && aspectRatio < 1.3) {
			center.x = cvRound((face.x + face.width * 0.5) * scale);
			center.y = cvRound((face.y + face.height * 0.5) * scale);
			radius = cvRound((face.width + face.height) * 0.25 * scale);
			cv::circle(frame, center, radius, color, 3, 8, 0);
		}
		else {
			cv::rectangle(frame, cv::Point(cvRound(face.x * scale), cvRound(face.y * scale)),
				cv::Point(cvRound((face.x + face.width - 1) * scale), 
				cvRound((face.y + face.height - 1) * scale)), color, 3, 8, 0);
		}
		smallFrameRoi = smallFrame(face);
		nestedCascade.detectMultiScale(smallFrameRoi, nestedObjects, 1.1, 2, cv::CASCADE_SCALE_IMAGE, 
			cv::Size(30, 30));
		for (cv::Rect& nested : nestedObjects) {
			center.x = cvRound((face.x + nested.x + nested.width * 0.5) * scale);
			center.y = cvRound((face.y + nested.y + nested.height * 0.5) * scale);
			radius = cvRound((nested.width + nested.height) * 0.25 * scale);
			cv::circle(frame, center, radius, color, 3, 8, 0);
		}
	}

	cv::imshow(window_name, frame);
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
		DetectAndDraw(camFrame, cascade, nestedCascade, scale);
		cv::pollKey();
	}
	if (cv::getWindowProperty(window_name, cv::WindowPropertyFlags::WND_PROP_VISIBLE)) {
		cv::destroyWindow("Face");
	}
}