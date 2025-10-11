#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


int main() {
    // 打开摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    cv::Mat frame;
	cv::Mat canny;
	cv::Mat blur;

    while (true) {
        // 读取一帧
        cap >> frame;
        if (frame.empty()) break;

		cv::blur(frame, blur, cv::Size(3,3));
        cv::Canny(blur, canny, 40,100);
        cv::imshow("Camera Feed", canny);

        // 按下 ESC 键退出
        if (cv::waitKey(30) == 27) break;
    }

    // 释放摄像头并关闭窗口
    cap.release();
    cv::destroyAllWindows();

    return 0;
}