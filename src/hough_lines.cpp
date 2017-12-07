#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#define IMAGE "rubik.jpg"
using namespace cv;
using namespace std;

const int th = 128; 
int bSize = 0;
Point bSum;
Mat imgo = imread(IMAGE,0);
Mat imgoc = imread(IMAGE,1);
Mat img(imgo.rows,imgo.cols,CV_8UC1);
float rmax = sqrt(img.rows*img.rows + img.cols*img.cols);
Mat imgh(2*rmax, 180, CV_8UC1, Scalar(0));
vector<Point> maximas;

bool isValid(int y, int x, Mat i)
{
	if(y < 0 || y >= i.rows)
		return false;
	else if(x < 0 || x >= i.cols)
		return false;
	else 
		return true;
}

void plotHough(int y, int x)
{
	for(int theta = 0; theta < 180; ++theta)
	{
		int r = x*cos(theta*3.14/180) + y*sin(theta*3.14/180);
		if(isValid(r+rmax,theta,imgh))
			if(imgh.at<uchar>(r+rmax,theta) < 255)
				imgh.at<uchar>(r+rmax,theta) += 1;
	}
}

void blobDetect(int y, int x, Mat imghv)
{
	cout << "\t" << y <<'\t' << x << endl;
	imghv.at<uchar>(y,x) = 255;
	Point current;
	current.y = y;
	current.x = x;
	bSum += current;
	bSize += 1;
	for(int j=-1;j<=1;++j)
	{
		for(int i=-1;i<=1;++i)
		{
			if(isValid(y+j,x+i,imgh))
				if(imgh.at<uchar>(y+j,x+i) >= th && imghv.at<uchar>(y+j,x+i) == 0)
					blobDetect(y+j,x+i,imghv);
		}
	}
}

void draw()
{
	cout << "Draw\n";
	for(int i=0;i<maximas.size();++i)
	{
		int r = maximas[i].y - rmax;
		float theta = maximas[i].x*3.14/180;
		for(int y=0;y<imgo.rows;++y)
		{
			int x = (r - y*sin(theta))/cos(theta);
			if(isValid(y,x,imgo))
			{
				imgoc.at<Vec3b>(y,x)[2] = 255;
				imgoc.at<Vec3b>(y,x)[1] = 0;
				imgoc.at<Vec3b>(y,x)[0] = 0;
			}
		}
		for(int x=0;x<imgo.cols;++x)
		{
			int y = (r - x*cos(theta))/sin(theta);
			if(isValid(y,x,imgo))
			{
				imgoc.at<Vec3b>(y,x)[2] = 255;
				imgoc.at<Vec3b>(y,x)[1] = 0;
				imgoc.at<Vec3b>(y,x)[0] = 0;
			}
		}
	}
	imshow("Hough Lines",imgoc);
}

void detectMaxima()
{
	Mat imghv(imgh.rows,imgh.cols,CV_8UC1,Scalar(0));
	for(int y=0;y<imgh.rows;++y)
	{
		for(int x=0;x<imgh.cols;++x)
		{
			if(imgh.at<uchar>(y,x) >= th && imghv.at<uchar>(y,x) != 255)
			{
				cout << "DM\n";
				bSize = 0;
				bSum.x = 0;bSum.y = 0;
				blobDetect(y,x,imghv);
				bSum.x /= bSize;
				bSum.y /= bSize;
				//circle(imghv, bSum, 3, Scalar(128));
				maximas.push_back(bSum);
			}
		}
	}
	draw();
}


int main()
{
	namedWindow("Hough Space",CV_WINDOW_AUTOSIZE);
	Canny(imgo,img,0,255,3,false);
	for(int y=0;y<img.rows;++y)
	{
		for(int x=0;x<img.cols;++x)
		{
			if(img.at<uchar>(y,x) >= 127)
				plotHough(y,x);
		}
	}
	imshow("Canny", img);
	detectMaxima();
	imshow("Hough Space", imgh);
	waitKey(0);
	return 0;
}


