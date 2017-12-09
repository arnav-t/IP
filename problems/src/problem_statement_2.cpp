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

int smin = 120;
int vmin = 180;
int vmin2 = 40;
Mat img;
Mat imgobj(720,1280,CV_8UC1,Scalar(0));
Mat imgbot(720,1280,CV_8UC1,Scalar(0));
Mat imgvis(720,1280,CV_8UC1,Scalar(0));
const int areaThreshold = 40;
const int botThreshold = 100;
const int distThreshold = 25;
const int yDistScale = 10;
void upImg()
{
	imgobj = Scalar(0);
	imgbot = Scalar(0);
	cvtColor(img,img,CV_BGR2HSV);
	for(int y=0;y<img.rows;++y)
	{
		for(int x=0;x<img.cols;++x)
		{
			Vec3b col = img.at<Vec3b>(y,x);
			if(((col[0] >= ymin && col[0] <= ymax) && col[1] >= smin) && col[2] >= vmin)
			{
				imgobj.at<uchar>(y,x) = 255;
			}
			if(((col[0] >= pmin && col[0] <= pmax) && col[1] >= smin) && col[2] >= vmin2)
			{
				imgbot.at<uchar>(y,x) = 255;
			}
		}
	}
}

int main()
{
	VideoCapture vid(VIDEO);
	vid >> img;
	int score = 0;
	int frameNum = 0;
	vector<Point> objCentroidsPerma;
	while(!img.empty())
	{
		vid >> img;
		//GaussianBlur(img, img, Size(7,7), 10);
		upImg();
		vector<Vec4i> hierarchy;
		vector<vector<Point>> contours;
		vector<vector<Point>> polygons;
		vector<Point> objCentroids;
		findContours(imgobj, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		for(int i=0;i<contours.size();++i)
		{
			if(contourArea(contours[i]) >= areaThreshold)
			{
				vector<Point> currContour;
				approxPolyDP(contours[i], currContour, 5, true);
				polygons.push_back(currContour);
				int hue = 0;
				int sat = 0;
				if(currContour.size() == 3)
				{
					drawContours(img, polygons, polygons.size() - 1, Scalar(0,255,255),1);
					sat = 255;
				}
				else if(currContour.size() == 4)
				{
					drawContours(img, polygons, polygons.size() - 1, Scalar(60,255,255),1);
					hue = 60;
					sat = 255;
				}
				for(int j=0;j<currContour.size();++j)
				{
					circle(img, currContour[j], 3, Scalar(hue,sat,255));
				}
				if(sat == 255)
				{
					Rect bounds = boundingRect(currContour);
					Point objCentroid(bounds.x + bounds.width/2,bounds.y + bounds.height/2);
					circle(img, objCentroid, 3, Scalar(hue,sat,255), CV_FILLED);
					objCentroids.push_back(objCentroid);
				}
				
			}
		}
		if(frameNum%20 == 0)
		{
			objCentroidsPerma = objCentroids;
		}
		contours.clear();
		polygons.clear();
		hierarchy.clear();
		findContours(imgbot, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		float maxArea = 0;
		Point centroid(0,0);
		for(int i=0;i<contours.size();++i)
		{
			float currArea = contourArea(contours[i]);
			if(currArea >= botThreshold)
			{
				bool isBot = false;
				if(currArea > maxArea)
				{
					isBot = true;
					maxArea = currArea;
				}
				vector<Point> currContour;
				approxPolyDP(contours[i], currContour, 5, true);
				polygons.push_back(currContour);
				if(isBot)
				{
					centroid.x = 0;
					centroid.y = 0;
					for(int j=0;j<currContour.size();++j)
						centroid += currContour[j];
					centroid.x /= currContour.size();
					centroid.y /= currContour.size();
				}
			}
		}
		circle(img, centroid, 3, Scalar(120,255,255), CV_FILLED);
		for(int k=0;k<objCentroids.size();++k)
		{
			if(abs(objCentroids[k].x - centroid.x) + abs(objCentroids[k].y - centroid.y) <= (distThreshold + objCentroids[k].y/yDistScale))
			{
				line(img, centroid, objCentroids[k], Scalar(30,255,255), 2, CV_AA);
				if(imgvis.at<uchar>(objCentroids[k].y,objCentroids[k].x) == 0)
					score += 100;
				circle(imgvis, objCentroids[k], 100, 255, CV_FILLED);
			}
			else if(abs(objCentroidsPerma[k].x - centroid.x) + abs(objCentroidsPerma[k].y - centroid.y) <= (distThreshold + objCentroidsPerma[k].y/yDistScale))
			{
				line(img, centroid, objCentroidsPerma[k], Scalar(0,255,255), 2, CV_AA);
				if(imgvis.at<uchar>(objCentroidsPerma[k].y,objCentroidsPerma[k].x) == 0)
					score += 100;
				circle(imgvis, objCentroidsPerma[k], 100, 255, CV_FILLED);
			}
		}
		putText(img, to_string(score) + " Points", Point(20,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0,255,255), 1, CV_AA);
		cvtColor(img, img, CV_HSV2BGR);
		imshow("Fortress",img);
		++frameNum;
		waitKey(1);
	}
	waitKey(0);
	return 0;
}


