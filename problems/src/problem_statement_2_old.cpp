#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <iostream>
#include <queue>
#define VIDEO "PS2.mp4"
using namespace cv;
using namespace std;

int ymax = 30;
int ymin = 20;
int pmax = 165;
int pmin = 155;
Mat img;
Mat imgobj(720,1280,CV_8UC1,Scalar(0));
const int areaThreshold = 100;
const int cornerThreshold = 5;
const float angleThreshold = CV_PI/3;

void upImg()
{
	imgobj = Scalar(0);
	cvtColor(img,img,CV_BGR2HSV);
	for(int y=0;y<img.rows;++y)
	{
		for(int x=0;x<img.cols;++x)
		{
			Vec3b col = img.at<Vec3b>(y,x);
			if((col[0] >= ymin && col[0] <= ymax) && col[1] >= 120)
			{
				imgobj.at<uchar>(y,x) = 255;
			}
			/*if((col[0] >= pmin && col[0] <= pmax) && col[1] >= 120)
			{
				imgobj.at<uchar>(y,x) = 127;
			}*/
		}
	}
	cvtColor(img,img,CV_HSV2BGR);
	//imshow("Fortress",img);
}

int main()
{
	VideoCapture vid(VIDEO);
	vid >> img;
	Mat img2(720,1280,CV_8UC3);
	while(!img.empty())
	{
		vid >> img;
		GaussianBlur(img, img, Size(3,3), 1);
		upImg();
		vector<Vec4i> hierarchy;
		vector<vector<Point>> contours;
		findContours(imgobj, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		img2 = Scalar(0,0,0);
		for(int i=0;i<contours.size();++i)
		{
			float area = contourArea(contours[i]);
			float side = sqrt(area);
			if(area >= areaThreshold)
			{
				vector<Point> currContour = contours[i];
				vector<float> angles;
				vector<Point> corners;
				for(int j=1;j<currContour.size();++j)
					angles.push_back(atan2(currContour[j].y-currContour[j-1].y,currContour[j].x-currContour[j-1].x));
				angles.push_back(atan2(currContour.back().y-currContour.front().y,currContour.back().x-currContour.front().x));
				if(abs(angles.back()-angles.front()) >= angleThreshold)
					corners.push_back(currContour.front());
				for(int j=1;j<angles.size();++j)
					if(abs(angles[j]-angles[j-1]) >= angleThreshold)
						corners.push_back(currContour[j]);
				priority_queue<int> eraseQ;
				if(corners.size() >= 2)
				{
					for(int j=1;j<corners.size();++j)
					{
						if(abs(corners[j].x - corners[j-1].x) + abs(corners[j].y - corners[j-1].y) <= side/cornerThreshold)
							eraseQ.push(j);
					}
					if(abs(corners.back().x - corners.front().x) + abs(corners.back().y - corners.front().y) <= side/cornerThreshold)
					{
						if(!eraseQ.empty())
							if(eraseQ.top() != corners.size() - 1)
								eraseQ.push(corners.size() - 1);
						else
							eraseQ.push(corners.size() - 1);
					}
					while(!eraseQ.empty())
					{
						corners[eraseQ.top()] = corners.back();
						corners.pop_back();
						eraseQ.pop();
					}
				}
				cout << corners.size() << endl;
				int hue = 0;
				int sat = 0;
				if(corners.size() == 3)
				{
					drawContours(img2, contours, i, Scalar(0,255,255),1);
					sat = 255;
				}
				else if(corners.size() == 4)
				{
					drawContours(img2, contours, i, Scalar(60,255,255),1);
					hue = 60;
					sat = 255;
				}
				else
					drawContours(img2, contours, i, Scalar(100,255,127),1);
				for(int j=0;j<corners.size();++j)
					circle(img2, corners[j], 3, Scalar(hue,sat,255));
			}
		}
		//Canny(imgobj,imgobj,100,200,3,false);
		cvtColor(img2, img2, CV_HSV2BGR);
		imshow("Objects",img2);
		waitKey(1);
	}
	return 0;
}


