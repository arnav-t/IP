/*
	Black is traversable
	White is not
*/
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <vector>
#define IMAGE "maze.jpg"

using namespace cv;
using namespace std;

struct node
{
	int y,x,len;
	node* parent;
	node(int Y=-1, int X=-1,int Len=0)
	{
		y = Y;
		x = X;
		len = Len;
		parent = nullptr;
	}
	int getLength()
	{
		if(parent == nullptr)
			return len;
		else if(x-parent->x && y-parent->y)
			return 14 + parent->getLength();
		else
			return 10 + parent->getLength();
	}
};

vector<node *> open;
/*vector<node *> close;

node *find(int y, int x)
{
	for(int i=0;i<close.size();++i)
	{
		if(close[i]->x == x && close[i]->y == y)
			return (close[i]);
	}
	return nullptr;
}*/
void find_pop(int y, int x)
{
	for(int i=0;i<open.size();++i)
	{
		if(open[i]->x == x && open[i]->y == y)
			open.erase(open.begin()+i);
	}
}

node *min()
{
	int minIndex = 0;
	int min = open[0]->getLength();
	for(int i=0;i<open.size();++i)
	{
		if(open[i]->getLength() < min)
		{
			min = open[i]->getLength();
			minIndex = i;
		}
	}
	return open[minIndex];
}

Mat img = imread(IMAGE,0);
Mat imgd = imread(IMAGE,1);
Mat imgc(img.rows,img.cols,CV_8UC1,Scalar(0));
Point i,f;
node *initNode;
int state = 0;

bool isValid(int y, int x)
{
	if(y >= img.rows || y < 0)
		return false;
	else if(x >= img.cols || x < 0)
		return false;
	else 
		return true;
}

void draw(node *a)
{
	node *i = a;
	while(i->parent != nullptr)
	{
		imgd.at<Vec3b>(i->y,i->x)[2] = 255;
		imgd.at<Vec3b>(i->y,i->x)[1] = 0;
		imgd.at<Vec3b>(i->y,i->x)[0] = 0;
		i = i->parent;
	}
}

void path(node *a)
{
	find_pop(a->y,a->x);
	imgc.at<uchar>(a->y,a->x) = 255;
	//imshow("Progress",imgc);
	//waitKey(1);
	for(int j=-1;j<=1;++j)
	{
		for(int i=-1;i<=1;++i)
		{
			if(isValid(a->y+j,a->x+i) && (i||j))
			{
				if(img.at<uchar>(a->y+j,a->x+i) < 128)
				{
					if(imgc.at<uchar>(a->y+j,a->x+i) < 128)
					{
						node *next = new node(a->y+j,a->x+i,((i&&j)?14:10) + a->getLength());
						next->parent = a;
						open.push_back(next);
					}
				}
			}
		}
	}
	if(f.x == a->x && f.y == a->y)
		draw(a);
	else
		path(min());
}

void init(int event, int x, int y, int flags, void* a)
{
	
	if(event == EVENT_LBUTTONDOWN)
	{
		if(state == 0)
		{
			i.x = x;
			i.y = y;
			state += 1;
			cout << "Initial Point Recorded (" << i.y << ", " << i.x << ")\n";
			circle(imgd, i, 3, Scalar(0,255,0));
			imshow("Dijsktra",imgd);
		}
		else if(state == 1)
		{
			f.x = x;
			f.y = y;
			state += 1;
			circle(imgd, f, 3, Scalar(255,0,0));
			imshow("Dijsktra",imgd);
			waitKey(1);
			cout << "Final Point Recorded (" << f.y << ", " << f.x << ")\n";
			initNode = new node(i.y,i.x,0);
			open.push_back(initNode);
			path(initNode);
			imshow("Dijsktra",imgd);
			imshow("Progress",imgc);
			waitKey(0);
		}
		else
			return;
	}
}

int main()
{
	namedWindow("Dijsktra",CV_WINDOW_AUTOSIZE);
	namedWindow("Progress",CV_WINDOW_AUTOSIZE);
	imshow("Progress",imgc);
	imshow("Dijsktra",imgd);
	setMouseCallback("Dijsktra", init, NULL);
	waitKey(0);
	return 0;
}


