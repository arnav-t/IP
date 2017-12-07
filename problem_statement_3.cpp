#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
using namespace cv;
using namespace std;

int hmax = 100; // max hue of target
int hmin = 80; // min hue of target
int en = 0; // 0 for hue adjustment, 1 for drawing mode, 2 for processing and displaying the resutls
VideoCapture vid(0);
Mat img;
Mat imgo;
Point bmax,bmin,avg;
int bSum = 0; // total number of blobs (not used)
int bSize = 0; // size of current blob
int bSizeMax = 0; // size of the largest blob
Point bMaxPoint; // centroid of the largest blob
const int bTh = 500; // threshold for blob size for it to be considered a suitable target
vector<Point> contour; // vector for storing centroid points during drawing
const float thres = 3.14/6; // angular resolution used in hough transform 

bool isValid(int y, int x, Mat i)
{
	if(y < 0 || y >= i.rows)
		return false;
	else if(x < 0 || x >= i.cols)
		return false;
	else 
		return true;
}


void blobDetect(int y,int x, Mat imgv)
{
	imgv.at<uchar>(y,x) = 255;
	Point current;
	current.y = y;
	current.x = x;
	avg += current;
	bSize += 1;
	if(x > bmax.x)
		bmax.x = x;
	if(y > bmax.y)
		bmax.y = y;
	if(x < bmin.x)
		bmin.x = x;
	if(y < bmin.y)
		bmin.y = y;
	for(int j=-1;j<=1;++j)
	{
		for(int i=-1;i<=1;++i)
		{
			if(isValid(y+j,x+i,img))
			{
				if((img.at<Vec3b>(y+j,x+i)[0] >= hmin && img.at<Vec3b>(y+j,x+i)[0] <= hmax) && imgv.at<uchar>(y+j,x+i) == 0)
					blobDetect(y+j,x+i, imgv);
			}
		}
	}
}

void upImg()
{
	bSizeMax = 0; // reset max blob size for each new fresh frame
	cvtColor(img,img,CV_BGR2HSV);
	cvtColor(imgo,imgo,CV_BGR2HSV);
	Mat imgv(img.rows,img.cols,CV_8UC1,Scalar(0));
	bSum = 0;
	for(int y=0;y<img.rows;++y)
	{
		for(int x=0;x<img.cols;++x)
		{
			Vec3b col = img.at<Vec3b>(y,x);
			if(en)
			{
				if((col[0] >= hmin && col[0] <= hmax) && imgv.at<uchar>(y,x) == 0)
				{
					bSum += 1;
					bSize = 0;
					avg.x = 0;
					avg.y = 0;
					bmax.x = x;
					bmin.x = x;
					bmax.y = y;
					bmin.y = y;
					blobDetect(y,x,imgv);
					avg.x /= bSize;
					avg.y /= bSize;
					if(bSizeMax < bSize)
					{
						bSizeMax = bSize;
						bMaxPoint = avg;
					}
					//cout << " "<< bSize;
					if(bSize >= bTh)
					{
						rectangle(imgo, bmax, bmin, Scalar(0,255,255));
						circle(imgo, avg, 3, Scalar(60,255,255),CV_FILLED);
						//cout << "Object Found\n";
					}
				}
			}
			else if(col[0] < hmin || col[0] > hmax)
			{
				imgo.at<Vec3b>(y,x)[1] = 0;
				imgo.at<Vec3b>(y,x)[2] /= 2;
			}
		}
	}
	if(bSizeMax)
		contour.push_back(bMaxPoint);
	cvtColor(img,img,CV_HSV2BGR);
	cvtColor(imgo,imgo,CV_HSV2BGR);
	imshow("HSV",imgo);
	//cout << bSum << endl;t
}

int main()
{
	namedWindow("HSV",CV_WINDOW_AUTOSIZE);
	Mat imgcont(480,640,CV_8UC1,Scalar(0));
	createTrackbar( "Min", "HSV", &hmin, 179, NULL);
	createTrackbar( "Max", "HSV", &hmax, 179, NULL);
	createTrackbar( "Enable", "HSV", &en, 2, NULL);
	vector<vector<Point>> contours;
	contours.push_back(contour);
	while(waitKey(300) && en!=2)
	{
		imgcont = Scalar(0,0,0);
		vid >> imgo;
		vid >> img;
		//GaussianBlur(img, img, Size(5,5), 1000);
		upImg();
		contours[0] = contour;
		if(bSizeMax >= bTh && en)
		{
			drawContours(imgcont, contours, 0, Scalar(255), 1);
			imshow("Contours", imgcont);
		}
	}
	vector<Vec2f> houghlines;
	HoughLines(imgcont, houghlines, 3, 0.017, 110);
	cout << "Hough Lines: " << houghlines.size() << endl;
	Point prev = contour[0];
	Point current;
	vector<float> angles;
	for(int i=1;i < contour.size();++i)
	{
		current = contour[i];
		float angle = atan2(current.y - prev.y, current.x - prev.x);
		angles.push_back(angle);
		prev = current;
	}
	int corner = 1;
	vector<Point> corners;
	circle(imgcont, contour[0], 5, 128);
	corners.push_back(contour[0]);
	for(int i=1;i<angles.size();++i)
	{
		current = contour[i];
		if(abs(angles[i-1]-angles[i]) >= thres)
		{
			if(abs(contour[i].x - corners.back().x) + abs(contour[i].y - corners.back().y) >= 40)
			{
				corners.push_back(contour[i]);
				circle(imgcont, contour[i], 5, 128);
				corner += 1;
			}
		}
	}
	putText(imgcont, to_string(corner) + " pointed polygon detected", Point(0,450), FONT_HERSHEY_COMPLEX_SMALL, 0.8, 255, 1, CV_AA);
	imshow("Contours", imgcont);
	cout << "Corner: " << corner << endl;
	waitKey(0);
	return 0;
}


