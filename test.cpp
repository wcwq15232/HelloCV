#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	Mat img(512, 512, CV_8UC3, Scalar(255, 255, 255));

	circle(img, Point(256, 256), 155, Scalar(0, 69, 255), -1);

	rectangle(img, Point(130, 226), Point(382, 286), Scalar(255, 255, 255), -1);
	imshow("image", img);

	waitKey(0);

}


