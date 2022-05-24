#ifndef DSPROJECT_MAP_H
#define DSPROJECT_MAP_H

#include <cassert>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "agent.h"
#include "config.h"

// buildings, landmarks or unnamed road segment
class Node {
   public:
    double x;  // longitude
    double y;  // latitude
    int idx;
    std::string name;
    explicit Node(double x, double y) : x(x), y(y), idx(-1), name() {}
    explicit Node(double x, double y, int idx) : x(x), y(y), idx(idx), name() {}
    explicit Node(double x, double y, int idx, std::string name)
        : x(x), y(y), idx(idx), name(name) {}
};

class Road {
   public:
    const int MAX_CAPACITY = 1000000;
    int u, v;  // undirected edge, guarantee that cur < v
    int idx;   // road index
    int capacity;
    double difficulty;  // [0, 1], velocity *= (1 - difficulty);
    int num_agents;     // currently the number of agents

    explicit Road(int u, int v, int idx);
    explicit Road(int u, int v, int idx, int capacity, double difficulty);
};

extern std::vector<Node> nodes;
extern std::vector<Road> roads;
extern std::vector<ReadEdge> walls;
extern std::map<std::string, int> mp_node_names;
extern std::vector<int> G[200];

void setBan1(bool x);
void setBan2(bool x);
bool getBan1();
bool getBan2();

double bfs(Agent* agent, int u, int t);
double aStar(int u, int t);

std::vector<int> getRouteOfWaypoints(int t);

void initMap();

double getMaxX();
double getMaxY();
double getMinX();
double getMinY();

#endif  // DSPROJECT_MAP_H
