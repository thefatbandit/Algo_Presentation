#include <bits/stdc++.h>
#include <SFML/Graphics.hpp>
#include "geometry.h"

using namespace std ; 

#define MAX_INT 100000000

const int WIDTH = 800 ;
const int HEIGHT = 600 ;
const int RADIUS = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const double JUMP_SIZE = (WIDTH/100.0 * HEIGHT/100.0)/1.5;
const double DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found 

int whichPlanner = 3 ; 

vector < Polygon > obstacles ; 
Point start, stop ; 
int obstacle_cnt = 1 ;

queue < Point > nodes; 
// vector < int > parent, nearby ;
 
int vis[WIDTH][HEIGHT] = {0};
int cost[WIDTH][HEIGHT] = {MAX_INT};
Point parent[WIDTH][HEIGHT];

int nodeCnt = 0, goalIndex = -1 ; 

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

	cout << "Which type of RRT would you like to watch? 1 for RRT, 2 for RRT*, 3 for Anytime RRT" << endl ;
	cin >> whichPlanner ; 
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

	// Draw obstacles 
	for(auto& poly : polygons) window.draw(poly);

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

	window.draw(startingPoint); window.draw(endingPoint);
	

	if (pathFound)
	{
		Point node = stop;
		while(!(parent[(int) node.x][(int) node.y]==node)){
			Point par = parent[(int) node.x][(int) node.y];
			line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
			line[1] = sf::Vertex(sf::Vector2f(node.x, node.y));
			line[0].color = line[1].color = sf::Color::Red; // orange color 
			window.draw(line, 2, sf::Lines);
			node = par ;
		}
	}
	// If destination is reached then path is retraced and drawn 
	// if(pathFound) {
	// 	int node = goalIndex; 
	// 	while(parent[node] != node) {
	// 		int par = parent[node];
	// 		line[0] = sf::Vertex(sf::Vector2f(nodes[par].x, nodes[par].y));
	// 		line[1] = sf::Vertex(sf::Vector2f(nodes[node].x, nodes[node].y));
	// 		line[0].color = line[1].color = sf::Color::Red; // orange color 
	// 		window.draw(line, 2, sf::Lines);
	// 		node = par ;
	// 	}
	// }
}

template <typename T> // Returns a random number in [low, high] 
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Point a, Point b) {
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ; 
    return true ; 
}

// Returns a random point with some bias towards goal 
Point pickRandomPoint() {
    double random_sample = randomCoordinate(0.0, 1.0); 
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + Point(RADIUS, RADIUS) ;
	return Point(randomCoordinate(0, WIDTH), randomCoordinate(0, HEIGHT)); 
}

// void checkDestinationReached() {
// 	sf::Vector2f position = endingPoint.getPosition(); 
// 	if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), RADIUS)) {
// 		pathFound = 1 ; 
// 		goalIndex = nodeCnt - 1;
// 		cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ; 
// 	}
// }

int isValid(Point par, Point p){
	if(p.x<0||p.y<0||p.y>HEIGHT||p.x>WIDTH) return 0;

	if(!isEdgeObstacleFree(par,p)) return 0;

	return 1;
}

void Dijkstra(){
	// Point newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ; 
	// int nearestIndex = 0 ; double minCost = INF; nearby.clear(); jumps.resize(nodeCnt);

	Point p = nodes.front();
	vis[(int) p.x][(int) p.y] = 1;

	for (int i = -1; i < 2; ++i)
	{
		for (int j = -1; j < 2; ++j)
		{
			if(i*j==0){
				Point temp;
				temp.x = p.x + i;
				temp.y = p.y + j;

				if(isValid(p,temp) and vis[(int) temp.x][(int) temp.y]!=1){
					nodes.push(temp);
					vis[(int) temp.x][(int) temp.y] = 1;
				}
				if((cost[(int) p.x][(int) p.y] + 1 < cost[(int) temp.x][(int) temp.y])){
				cost[(int) temp.x][(int) temp.y] = cost[(int) p.x][(int) p.y] + 1;
				parent[(int) p.x+i][(int) p.y+j] = {p.x,p.y};

				if(p==stop) pathFound = 1; 
				}
			}
			
		}
	}

	nodes.pop();
}

int callDijkstra(){
	sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Dijkstra");

	nodeCnt = 1; int iterations = 0 ; 

	// Setting the source as it's own parent & it's cost as 0
	parent[(int) start.x][(int) start.y] = {start.x,start.y};
	cost[(int) start.x][(int) start.y] = 0;

	// Pushing the source to the queue
	nodes.push({start.x,start.y});

    sf::Time delayTime = sf::milliseconds(5);

    while (window.isOpen() and !nodes.empty())
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
        if(!pathFound){
	        Dijkstra(); iterations++;
        }

        if(iterations % 25 == 0){
        	cout<<nodes.front().x<<" "<<nodes.front().y<<endl;
        }
        
		if(iterations % 1000 == 0) {
			cout << "Iterations: " << iterations << endl ; 
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
			cout << endl ;
		}

		//sf::sleep(delayTime);
		window.clear();
		draw(window); 
        window.display();
    }

}

signed main() {
	getInput(); prepareInput(); 
    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 


   	if(whichPlanner==4)	callDijkstra();

   	else cout<<"Please select a valid planner";
}