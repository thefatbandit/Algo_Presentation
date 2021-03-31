#include <bits/stdc++.h>
#include <SFML/Graphics.hpp>
#include "geometry_a_star.h"

using namespace std ; 

const int WIDTH = 800 ;
const int HEIGHT = 600 ;
const int RADIUS = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const int JUMP_SIZE = 1 ;
const double DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found 

int whichPlanner = 4 ; 

vector < Polygon > obstacles ; 
Point start, stop ; 
int obstacle_cnt = 1 ;

priority_queue < Point > nodes; 

// For keeping track of visited nodes 
int vis[WIDTH][HEIGHT] = {0};

// For storing costs & parents of nodes
float cost[WIDTH][HEIGHT];
Point parent[WIDTH][HEIGHT];
Point goal; 

vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ; 
bool pathFound = 0 ;

// Geting Initial Inputs
void getInput() {
	/*
		Takes Input for the Screen Size along with the obstacle positions 
	*/
	cout << "NOTE:" << endl ; 
	cout << "Height of screen: " << HEIGHT << " pixels." ;
	cout << " Width of screeen: " << WIDTH << " pixels." << endl ;
	cout << "Maximum distance by which algorithm jumps from one point to another: " << JUMP_SIZE << " units" << endl ;
	cout << "If you would like to change of any of these, please make modifications in code" << endl ; 
	cout << "Please provide your inputs keeping this in mind. " << endl << endl ;

	// cout << "Which type of RRT would you like to watch? 1 for RRT, 2 for RRT*, 3 for Anytime RRT" << endl ;
	// cin >> whichPlanner ; 
	cout << "Input co-ordinates of starting and ending point respectively in this format X1 Y1 X2 Y2" << endl ;
	cin >> start.x >> start.y >> stop.x >> stop.y ;
	cout << "How many obstacles?" << endl ; 
	cin >> obstacle_cnt ; 
	
	obstacles.resize(obstacle_cnt); 
	int pnts = 0 ; Point pnt ; 
	vector < Point > poly ; 
	
	for(int i = 0; i < obstacle_cnt; i++) {
		poly.clear();
		cout << "How many points in " << i+1 << "th polygon?" << endl ; 
		cin >> pnts ; 
		poly.resize(pnts);

		cout << "Input co-ordinates of " << i+1 << "th polygon in clockwise order" << endl ;
		for(int j = 0; j < pnts; j++) {
			cin >> pnt.x >> pnt.y ; 
			obstacles[i].addPoint(pnt);
		}
	}
}

// Prepares SFML objects of starting, ending point and obstacles 
void prepareInput() {
	// Make starting and ending point circles ready 
	startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS); 
    startingPoint.setFillColor(sf::Color(208, 0, 240)); endingPoint.setFillColor(sf::Color::Blue);
    startingPoint.setPosition(start.x, start.y); endingPoint.setPosition(stop.x, stop.y);
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);

    // Prepare polygon of obstacles 
	polygons.resize(obstacle_cnt);
	for(int i = 0; i < obstacle_cnt; i++) {
		polygons[i].setPointCount(obstacles[i].pointCnt); 
		polygons[i].setFillColor(sf::Color(89, 87, 98)); 
		for(int j = 0; j < obstacles[i].pointCnt; j++) 
			polygons[i].setPoint(j, sf::Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
	}
}

void draw(sf::RenderWindow& window) {
	sf::Vertex line[2]; sf::CircleShape nodeCircle;

	// Uncomment if circular nodes are to be drawn 
	/*
	for(auto& node: nodes) {
		nodeCircle.setRadius(RADIUS/2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
		nodeCircle.setOrigin(RADIUS/2.5, RADIUS/2.5); 
		nodeCircle.setFillColor(sf::Color(0, 255, 171)); nodeCircle.setPosition(node.x, node.y); 
		window.draw(nodeCircle);
	}
	*/


	for (int i = 0; i < WIDTH; ++i)
	{
		for (int j = 0; j < HEIGHT; ++j)
		{
			if(vis[i][j]){
				Point node = {(double) i,(double) j};
				Point par = parent[i][j];

				line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
				line[1] = sf::Vertex(sf::Vector2f(i, j));
				window.draw(line, 2, sf::Lines);
			}	
		}
	}

	// Draw obstacles 
	for(auto& poly : polygons) window.draw(poly);

	window.draw(startingPoint); window.draw(endingPoint);
	

	if (pathFound)
	{
		line[0] = sf::Vertex(sf::Vector2f(goal.x, goal.y));
		line[1] = sf::Vertex(sf::Vector2f(stop.x, stop.y));
		line[0].color = line[1].color = sf::Color::Red; // orange color 
		window.draw(line, 2, sf::Lines);

		Point node = goal;
		while(!(parent[(int) node.x][(int) node.y]==node)){
			Point par = parent[(int) node.x][(int) node.y];
			line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
			line[1] = sf::Vertex(sf::Vector2f(node.x, node.y));
			line[0].color = line[1].color = sf::Color::Red; // orange color 
			window.draw(line, 2, sf::Lines);
			node = par;
		}

	}
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Point a, Point b) {
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ; 
    return true ; 
}

// Checks whether the given node is within JUMP_SIZE distance from destination
bool checkDestinationReached(Point p){
	if(distance(p,stop) < (3*JUMP_SIZE)){
		return 1;
	}
	else{
		return 0;
	}
}

// Validates a new point
int isValid(Point par, Point p){
	if(p.x<0||p.y<0||p.y>HEIGHT||p.x>WIDTH) return 0;

	if(!isEdgeObstacleFree(par,p)) return 0;

	return 1;
}

float heuristic(Point p){
	return distance(p,start) +  distance(p,stop);
}

// Single Iteration Function for Djikstra
void aStar(){
	Point p = nodes.top();
	vis[(int) p.x][(int) p.y] = 1;

	if(checkDestinationReached(p)){
		pathFound = 1;
		goal = p;
	} 

	else{
		for (int i = -1; i < 2; ++i)
		{
			for (int j = -1; j < 2; ++j)
			{
				if(i*j==0){
					Point temp;
					temp.x = p.x + (i*JUMP_SIZE);
					temp.y = p.y + (j*JUMP_SIZE);
	
					if (isValid(p,temp))
					{
						if(vis[(int) temp.x][(int) temp.y]!=1){
							temp.hr = heuristic(temp);
							nodes.push(temp);
							vis[(int) temp.x][(int) temp.y] = 1;
						}
						if((cost[(int) p.x][(int) p.y] + JUMP_SIZE < cost[(int) temp.x][(int) temp.y])){
						cost[(int) temp.x][(int) temp.y] = cost[(int) p.x][(int) p.y] + JUMP_SIZE;
						parent[(int) temp.x][(int) temp.y] = {p.x,p.y};
						}
					}
				}

				else{
					Point temp;
					temp.x = p.x + (i*JUMP_SIZE);
					temp.y = p.y + (j*JUMP_SIZE);
	
					if (isValid(p,temp))
					{
						if(vis[(int) temp.x][(int) temp.y]!=1){
							nodes.push(temp);
							vis[(int) temp.x][(int) temp.y] = 1;
						}
						if((cost[(int) p.x][(int) p.y] + (1.414*JUMP_SIZE) < cost[(int) temp.x][(int) temp.y])){
						cost[(int) temp.x][(int) temp.y] = cost[(int) p.x][(int) p.y] + (1.414*JUMP_SIZE);
						parent[(int) temp.x][(int) temp.y] = {p.x,p.y};
						}
					}
				}

			}
		}
		nodes.pop();
	}
}

// Main Function Call
signed main() {
	getInput(); prepareInput(); 
    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 

    for (int i = 0; i < WIDTH; ++i)
    {
    	for (int j = 0; j < HEIGHT; ++j)
    	{
    		cost[i][j]= (HEIGHT*WIDTH)+200;
    	}
    }

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "A*");

	int iterations = 0,result=0; 

	// Setting the source as it's own parent & it's cost as 0
	parent[(int) start.x][(int) start.y] = {start.x,start.y};
	cost[(int) start.x][(int) start.y] = 0;

    sf::Time delayTime = sf::milliseconds(0.5);

	// Setting source heuristic & pushing it to queue
    start.hr = heuristic(stop);
	nodes.push(start);

    while ((window.isOpen() and !nodes.empty()))
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
            	window.close();
            	return 0; exit(0);
            }
        }

        aStar(); iterations++;
        if(pathFound & result==0){
        	float final_cost = cost[(int) goal.x][(int) goal.y] + distance(goal,stop);
		    cout << "Distance: " << final_cost << " units." << endl;
		    result=1;
        }

		window.clear();
		draw(window); 
        window.display();
    }

}