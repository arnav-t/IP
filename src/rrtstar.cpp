#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stack>
#include <cmath>
#include <cstdlib>
#include <iostream>
#define SIZE 512
using namespace cv;
using namespace std;
const int stepThreshold = 4;
const int traceThreshold = 8;
const int nbdThreshold = 12;
const int maxNodes = 12000;
const int maxCollisions = 500;

Mat img(SIZE,SIZE,CV_8UC3,Scalar(0,0,0));
Mat imgg(SIZE,SIZE,CV_8UC1,Scalar(0));
int state = 0;
Point start, finish, current, previous;
bool drawing = false;
bool stop = false;
int collisions = 0;

class Branch
{
	private:
		Point location;
		Branch *parent;
		vector<Branch *> children;
		int cost;
	public:
		Branch(Point l)
		{
			location = l;
			parent = nullptr;
			cost = 0;
		}
		void setParent(Branch *p)
		{
			parent = p;
			Point loc = p->getLocation();
		}
		Branch *getParent()
		{
			return parent;
		}
		Point getLocation()
		{
			return location;
		}
		int getCost()
		{
			if(parent == nullptr)
			{
				cost = 0;
				return 0;
			}
			Point pL = parent->getLocation();
			cost = abs(location.y - pL.y) + abs(location.x - pL.x) + parent->getCost();
			return cost;
		}
		Branch *addChild(Point l)
		{
			Point lAdd;
			if(abs(location.y - l.y) + abs(location.x - l.x) <= stepThreshold)
			{
				if(l == location)
				{
					l.x += pow(-1,rand());
					l.y += pow(-1,rand());
					collisions += 1;
					if(collisions > maxCollisions)
						stop = true;
				}	
				lAdd = l;
			}
			else
			{
				float mag = sqrt(pow((l.x - location.x),2) + pow((l.y - location.y),2));
				lAdd.x = location.x + (l.x - location.x)*stepThreshold/mag;
				lAdd.y = location.y + (l.y - location.y)*stepThreshold/mag;
			}
			if(imgg.at<uchar>(lAdd.y,lAdd.x) < 128)
			{
				Branch *newChild = new Branch(lAdd);
				newChild->setParent(this);
				children.push_back(newChild);
				newChild->getCost();
				return newChild;
			}
			return nullptr;
		}
		void appendChild(Branch *child)
		{
			children.push_back(child);
		}
		void removeChild(Point l)
		{
			for(int i=0;i<children.size();++i)
				if(l == children[i]->getLocation())
				{
					children.erase(children.begin() + i);
					break;
				}
		}
		Branch *getClosest(Point l)
		{
			Branch *closest = this;
			int minDist = abs(location.y - l.y) + abs(location.x - l.x);
			for(int i=0;i<children.size();++i)
			{
				Branch *closestChild = children[i]->getClosest(l);
				Point loc = closestChild->getLocation();
				int childMinDist = abs(loc.y - l.y) + abs(loc.x - l.x);
				if(childMinDist < minDist)
				{
					closest = closestChild;
					minDist = childMinDist;
				}
			}
			return closest;
		}
		void setCheapest(Branch *node, Branch *originalNode);
		void drawBranch()
		{
			for(int i=0;i<children.size();++i)
			{
				line(img, location, children[i]->getLocation(), Scalar(50,20,0), 1, CV_AA);
				children[i]->drawBranch();
			}
			//circle(img, location, 1, Scalar(0,25,25),CV_FILLED);
		}
		void traceParent()
		{
			if(parent != nullptr)
			{
				line(img, location, parent->getLocation(), Scalar(0,0,255), 1, CV_AA);
				parent->traceParent();
			}
		}
};

stack<Branch *> neighbours;
void Branch::setCheapest(Branch *node, Branch *originalNode)
{
	Point l = node->getLocation();
	Point o = originalNode->getLocation();
	if(abs(location.y - o.y) + abs(location.x - o.x) < nbdThreshold && node->getLocation() == originalNode->getLocation())
		if(node->getLocation() != location)
			neighbours.push(this);
	if(parent == nullptr || abs(location.y - o.y) + abs(location.x - o.x) >= nbdThreshold)
	{
		for(int i=0;i<children.size();++i)
			children[i]->setCheapest(node,originalNode);
		return;
	}
	else
	{
		int nDist = abs(location.y - l.y) + abs(location.x - l.x);
		if(nDist + getCost() < node->getCost())
		{
			if(node->getParent() != nullptr)
			{
				node->getParent()->removeChild(node->getLocation());
				node->setParent(this);
				this->appendChild(node);
			}
		}
	}
	for(int i=0;i<children.size();++i)
	{
		Branch *cheapestChild = children[i]->getClosest(l);
		Point loc = cheapestChild->getLocation();
		int childMinCost = abs(loc.y - l.y) + abs(loc.x - l.x) + cheapestChild->getCost();
		if(abs(loc.y - o.y) + abs(loc.x - o.x) < nbdThreshold)
			if(childMinCost < node->getCost())
			{
				node->getParent()->removeChild(node->getLocation());
				node->setParent(cheapestChild);
				cheapestChild->appendChild(node);
				cheapestChild->setCheapest(node,originalNode);
			}
	}
}


void init(int event, int x, int y, int flags, void* a)
{
	current = Point(x,y);
	if(event == EVENT_RBUTTONDOWN)
	{
		if(state == 0)
		{
			state += 1;
			start = Point(x,y);
			circle(img, start, 3, Scalar(0,255,0),CV_FILLED);
			imshow("RRT*",img);
			cout << "Start point registered at (" << y  << ", " << x << ")\n";
			waitKey(1);
		}
		else if(state == 1)
		{
			state += 1;
			finish = Point(x,y);
			circle(img, finish, 3, Scalar(0,0,255),CV_FILLED);
			imshow("RRT*",img);
			cout << "Finish point registered at (" << y  << ", " << x << ")\n";
			waitKey(1);
			cout << "Creating tree...\n";
			Branch *tree = new Branch(start);
			for(int j=0;j<maxNodes && !stop;++j)
			{
				Branch *closestNode = nullptr;
				Branch *newChild = nullptr;
				while(newChild == nullptr)
				{
					Point randomPoint(rand()%SIZE,rand()%SIZE);
					closestNode = tree->getClosest(randomPoint);
					newChild = closestNode->addChild(randomPoint);
				}
				if(newChild != nullptr)
				{
					while(!neighbours.empty()) neighbours.pop();
					tree->setCheapest(newChild,newChild);
					//cout << neighbours.size() << endl;
					while(!neighbours.empty())
					{
						tree->setCheapest(neighbours.top(),newChild);
						neighbours.pop();
					}
					if(j%500 == 0)
					{
						img = Scalar(0,0,0);
						for(int y=0;y<SIZE;++y)
							for(int x=0;x<SIZE;++x)
								if(imgg.at<uchar>(y,x) == 255)
									img.at<Vec3b>(y,x) = Vec3b(255,255,255);
						tree->drawBranch();
						circle(img, start, 3, Scalar(0,255,0),CV_FILLED);
						circle(img, finish, 3, Scalar(0,0,255),CV_FILLED);
						imshow("RRT*",img);
						waitKey(1);
					}
				}
			}
			Branch *lastNode = tree->getClosest(finish);
			Point lastLoc = lastNode->getLocation();
			if(abs(lastLoc.y - finish.y) + abs(lastLoc.x - finish.x) <= traceThreshold)
				lastNode->traceParent();
			imshow("RRT*",img);
			cout << "Process finished.\n"; 
			waitKey(0);
		}
	}
	else if(event == EVENT_LBUTTONDOWN)
	{
		drawing = true;
		previous = current;
	}
	else if(event == EVENT_LBUTTONUP)
		drawing = false;
	if(drawing)
	{
		line(img, previous, current, Scalar(255,255,255), 10, CV_AA);
		line(imgg, previous, current, 255, 10, CV_AA);
		previous = current;
		imshow("RRT*",img);
		waitKey(1);
	}
	
}

int main()
{
	srand(time(NULL));
	namedWindow("RRT*",CV_WINDOW_AUTOSIZE);
	imshow("RRT*",img);
	waitKey(1);
	setMouseCallback("RRT*", init, NULL);
	waitKey(0);
	return 0;
}


