#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#define IMAGE "maze.jpg"
using namespace cv;
using namespace std;

Mat img = imread(IMAGE,1);
Mat imgg = imread(IMAGE,0);
Mat imgv(img.rows,img.cols,CV_8UC1,Scalar(0));

template <class T>
bool isValid(T y, T x)
{
	if(y < 0 || y >= img.rows)
		return false;
	else if(x < 0 || x >= img.cols)
		return false;
	else
		return true;
}

void castRay(Point source, float theta)
{
	float y = source.y;
	float x = source.x;
	float dx = cos(CV_PI*theta/180);
	float dy = sin(CV_PI*theta/180);
	float r = 1; 
	while(isValid(y,x))
	{
		if(imgv.at<uchar>(y,x) == 0)
		{
			r = 1 + sqrt(pow(x-source.x,2)+pow(y-source.y,2))/100;
			if(imgg.at<uchar>(y,x) < 128)
				img.at<Vec3b>(y,x) = Vec3b(0,255/pow(r,2),0);
			else
				break;
			imgv.at<uchar>(y,x) = 255;
		}
		x += dx;
		y += dy;
	}
}

void upImg(int event, int x, int y, int flags, void* a)
{
	img = imread(IMAGE,1);
	imgv = Scalar(0);
	for(float i=0;i<360;i+=0.1)
		castRay(Point(x,y),i);
	imshow("Light",img);
	waitKey(1);
}

int main()
{
	namedWindow("Light",CV_WINDOW_AUTOSIZE);
	imshow("Light",img);
	setMouseCallback("Light", upImg, NULL);
	waitKey(0);
	return 0;
}


