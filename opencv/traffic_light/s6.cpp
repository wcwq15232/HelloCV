#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

using namespace std;
using namespace cv;

bool getContours(Mat& imgDil, Mat& img){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // drawContours(img, contours, -1, Scalar(255, 0, 255), 2);
    bool result = false;

    for (int i = 0; i < contours.size(); ++i){
        int area = contourArea(contours[i]);

        vector<vector<Point>> conPoly(contours.size());
        vector<Rect> boundRect(contours.size());
        string objectType = "";
        
        if (area > 30000){
            result = true;
            float peri = arcLength(contours[i], true);
            approxPolyDP(contours[i], conPoly[i], 0.03 * peri, true);
            drawContours(img, conPoly, i, Scalar(0, 0, 255), 2, LINE_4);
            cout << contours[i].size() << endl;
            boundRect[i] = boundingRect(conPoly[i]);
            rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2);
        }
    }
    return result;
}

vector<vector<int>> RG {{133,120,150,179,255,255}, {74, 144, 127, 93, 251, 218}};
vector<Scalar> BGR {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
map<string, int> map_{{"blue", 0}, {"green", 1}, {"red", 2}};

int main(){
    string path = "TrafficLight.mp4", objType;
    VideoCapture cap(path);

    // double fps = cap.get(CAP_PROP_FPS);
    // Size frameSize(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT));
    // VideoWriter writer("output_f.mp4", VideoWriter::fourcc('M', 'P', '4', 'V'), fps, frameSize);

    Mat frame, frame_src;
    Mat imgHSV;
    Mat mask_r, mask_y, mask_g, canny_r, canny_y, canny_g;
    Mat kernel = getStructuringElement(cv::MORPH_RECT, Size(3,3));
    bool R, G;

    Scalar lower_r(RG[0][0], RG[0][1], RG[0][2]);
    Scalar upper_r(RG[0][3], RG[0][4], RG[0][5]);
    Scalar lower_g(RG[1][0], RG[1][1], RG[1][2]);
    Scalar upper_g(RG[1][3], RG[1][4], RG[1][5]);
    int re = -1;
    while(re != 27){
        cap.read(frame_src);
        objType = "";

        if (frame_src.empty()) break;

        GaussianBlur(frame_src, frame, Size(7, 7), 0);
        cvtColor(frame, imgHSV, COLOR_BGR2HSV);
        inRange(imgHSV, lower_r, upper_r, mask_r);
        inRange(imgHSV, lower_g, upper_g, mask_g);
        Canny(mask_r, canny_r, 75, 150);
        Canny(mask_g, canny_g, 75, 150);
        dilate(canny_r, canny_r, kernel);
        dilate(canny_g, canny_g, kernel);

        if(getContours(canny_r, frame_src)) objType = "red";
        if(getContours(canny_g, frame_src)) objType = "green";

        if (!objType.empty()) putText(frame_src, objType, Point(20, 110), FONT_HERSHEY_DUPLEX, 5, BGR[map_[objType]], 7);

        imshow("src", frame_src);
        re = waitKey(60);
        // writer.write(frame_src);
    }
    cap.release();
    // writer.release();
    destroyAllWindows();
}