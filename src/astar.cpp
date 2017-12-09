#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>
#define IMAGE "maze.jpg"
using namespace cv;
using namespace std;

Mat img = imread(IMAGE,1);
Mat imgg = imread(IMAGE,0);
int state = 0;
Point start, finish;

bool isValid(Point p)
{
	if(p.y >= img.rows || p.y < 0)
		return false;
	else if(p.x >= img.cols || p.x < 0)
		return false;
	else 
		return true;
}

class Node
{
	private:
		Point location;
		Node *parent;
		int weight;
		int heuristicDist()
		{
			return(abs(finish.y - location.y) + abs(finish.x - location.x));
		}
	public:
		int heuristicWeight;
		Node(Point l = Point(0,0))
		{
			location = l;
			parent = nullptr;
			weight = 0;
			heuristicWeight = 0;
		}
		int getWeight()
		{
			return weight;
		}
		void setLocation(Point l)
		{
			location = l;
		}
		void setParent(Node *p)
		{
			parent = p;
			weight = getTrueWeight();
			heuristicWeight = heuristicDist();
		}
		void traceToParent()
		{
			img.at<Vec3b>(location.y,location.x) = Vec3b(0,0,255);
			if(parent != nullptr)
				parent->traceToParent();
		}
		int getTrueWeight()
		{
			if(parent == nullptr)
				return 0;
			else if((parent->getLocation() - location).x && (parent->getLocation() - location).y)
				return 14 + parent->getTrueWeight();
			else
				return 10 + parent->getTrueWeight();
		}
		int getHeuristicWeight()
		{
			return
				heuristicDist() + weight;
		}
		Point getLocation()
		{
			return location;
		}
};

inline bool operator<(const Node& l, const Node& r)
{
    return l.heuristicWeight > r.heuristicWeight;
}

vector<Node> closed;
priority_queue<Node, vector<Node>> open;

bool searchClosed(Point p)
{
	for(int i=0;i<closed.size();++i)
		if(closed[i].getLocation() == p)
			return true;
	return false;
}

void aStar(Node p)
{
	open.pop();
	Point currPoint = p.getLocation();
	cout << "(" << currPoint.y  << ", " << currPoint.x << ") = " << p.heuristicWeight << endl;
	if(searchClosed(currPoint))
	{
		aStar(open.top());
		return;
	}
	img.at<Vec3b>(currPoint.y,currPoint.x) = Vec3b(63,63,63);
	imshow("A*",img);
	waitKey(1);
	if(currPoint == finish)
	{
		p.traceToParent();
		return;
	}
	closed.push_back(p);
	for(int j=-1;j<=1;++j)
	{
		for(int i=-1;i<=1;++i)
		{
			if(isValid(Point(currPoint.x + i, currPoint.y + j)))
				if(i || j)
					if((!searchClosed(Point(currPoint.x + i, currPoint.y + j))) && imgg.at<uchar>(currPoint.y + j,currPoint.x + i) < 128)
					{
						Node *next = new Node(Point(currPoint.x + i, currPoint.y + j));
						next->setParent(&p);
						open.push(*next);
					}
		}
	}
	if(!open.empty())
		aStar(open.top());
}

void init(int event, int x, int y, int flags, void* a)
{
	if(event == EVENT_LBUTTONDOWN)
	{
		if(state == 0)
		{
			state += 1;
			start = Point(x,y);
			circle(img, start, 3, Scalar(0,255,0),CV_FILLED);
			imshow("A*",img);
			cout << "Start point registered at (" << y  << ", " << x << ")\n";
			waitKey(1);
		}
		else if(state == 1)
		{
			state += 1;
			finish = Point(x,y);
			circle(img, finish, 3, Scalar(0,0,255),CV_FILLED);
			imshow("A*",img);
			cout << "Finish point registered at (" << y  << ", " << x << ")\n";
			waitKey(1);
			Node *startNode = new Node(start);
			cout << "Starting A* path planning...\n";
			open.push(*startNode);
			aStar(*startNode);
			circle(img, start, 3, Scalar(0,255,0),CV_FILLED);
			circle(img, finish, 3, Scalar(0,0,255),CV_FILLED);
			imshow("A*",img);
			cout << "Process finished.\n"; 
			waitKey(0);
		}
	}
}

int main()
{
	namedWindow("A*",CV_WINDOW_AUTOSIZE);
	imshow("A*",img);
	waitKey(1);
	setMouseCallback("A*", init, NULL);
	waitKey(0);
	return 0;
}


