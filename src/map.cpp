#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "agent.h"
#include "config.h"
#include "map.h"
#include "reader.h"
#include <QDebug>

// whether bridge 1/2 is banned
bool ban1 = false, ban2 = false;

// astar
int pre[200];
double dist[200];

std::vector<ReadNode> real_nodes;
std::vector<ReadEdge> real_roads;

std::map<std::string, int> mp_node_names;
std::vector<Node> nodes;
std::vector<Road> roads;
std::vector<ReadEdge> walls;
std::vector<int> G[200];
std::vector<Agent *> buildings[200];

Node real_node0(180, 90), real_node1(0, 0);
std::pair<double, double> pair0, pair1;

const double src_x_min = -100, src_x_max = 100;
const double src_y_min = -150, src_y_max = 150;

double x_max = -3000, x_min = 3000;
double y_max = -3000, y_min = 3000;

void setBan1(bool x) { ban1 = x; }
void setBan2(bool x) { ban2 = x; }
bool getBan1() { return ban1; }
bool getBan2() { return ban2; }

double deg2rad(double deg) { return deg * PI / 180; }

// https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
double calcDistance(const ReadNode &node1, const ReadNode &node2) {
  double diff_y = deg2rad(node2.second - node1.second);
  double diff_x = deg2rad(node2.first - node1.first);
  double a = sin(diff_y / 2) * sin(diff_y / 2) +
             cos(deg2rad(node1.second)) * cos(deg2rad(node2.second)) *
                 sin(diff_x / 2) * sin(diff_x / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = R * c;
  return d * 1000;
}

std::pair<double, double> convert_xy(double x, double y) {
  return std::make_pair(R * x * cos((real_node0.y + real_node1.y) * 0.5),
                        R * y);
}

std::pair<double, double> convert_node(double longitude, double latitude) {
  auto pair_temp = convert_xy(longitude, latitude);
  double per_x = (pair_temp.first - pair0.first) / (pair1.first - pair0.first);
  double per_y =
      (pair_temp.second - pair0.second) / (pair1.second - pair0.second);
  return std::make_pair(src_x_min + (src_x_max - src_x_min) * per_x,
                        src_y_min + (src_y_max - src_y_min) * per_y);
}

double h(int s, int t) { return calcDistance(real_nodes[s], real_nodes[t]); }

Road::Road(int u, int v, int idx)
    : idx(idx), capacity(MAX_CAPACITY), difficulty(1), num_agents(0) {
  assert(u < v);
  this->u = u, this->v = v;
  int width = 10;
  capacity = h(u, v) * width * 3;
}

Road::Road(int u, int v, int idx, int capacity, double difficulty)
    : idx(idx), num_agents(0) {
  assert(capacity >= 0 && capacity <= MAX_CAPACITY);
  assert(difficulty >= 0 && difficulty <= 1);
  assert(u < v);
  this->u = u, this->v = v, this->capacity = capacity,
  this->difficulty = difficulty;
  int width = 10;
  capacity = h(u, v) * width * 3;
}

double bfs(Agent *agent, int s, int t) {
  struct QueueStruct {
    int u;
    double cost;
    explicit QueueStruct(int u, double cost) : u(u), cost(cost) {}
  };

  std::queue<QueueStruct> q;
  for (int i = 0; i < 200; i++)
    dist[i] = 1e9;
  memset(pre, 0, sizeof(pre));
  dist[s] = 0;
  q.push(QueueStruct(s, dist[s]));
  while (!q.empty()) {
    QueueStruct temp = q.front();
    q.pop();
    int u = temp.u;
    if (u == t) {
      return dist[t];
    }
    for (int i : G[u]) {
      if (getBan1() && (i == 60 - 1 || i == 61 - 1 || i == 62 - 1))
        continue;
      if (getBan2() && (i == 81 - 1 || i == 82 - 1 || i == 83 - 1))
        continue;
      auto edge = roads[i];
      int v = edge.u;
      if (v == u)
        v = edge.v;

      double velocity = agent->getDesiredSpeed();
      velocity *= edge.difficulty;
      double ratio = 1.0 * edge.num_agents / edge.capacity;
      if (ratio > 0.7) {
        // choose a linearly decreasing function
        double temp = -7.0 / 3.0 * ratio + 79.0 / 30.0;
        assert(temp > 0);
        velocity *= temp;
      }
      double w =
          calcDistance(real_nodes[edge.u], real_nodes[edge.v]) / velocity;
      if (dist[v] > dist[u] + w) {
        dist[v] = dist[u] + w;
        pre[v] = u;
        q.push(QueueStruct(v, dist[v]));
      }
    }
  }
  return 1e9;
}

double aStar(int s, int t) {
  struct HeapStruct {
    int u;
    double cost;
    explicit HeapStruct(int u, double cost) : u(u), cost(cost) {}
    bool operator<(const HeapStruct &rhs) const { return cost > rhs.cost; }
  };

  std::priority_queue<HeapStruct> heap;
  for (int i = 0; i < 200; i++)
    dist[i] = 1e9;
  memset(pre, 0, sizeof(pre));
  dist[s] = 0;
  heap.push(HeapStruct(s, dist[s] + h(s, t)));
  while (!heap.empty()) {
    HeapStruct temp = heap.top();
    heap.pop();
    int u = temp.u;
    if (u == t) {
      return dist[t];
    }
    for (int i : G[u]) {
      if (getBan1() && (i == 60 - 1 || i == 61 - 1 || i == 62 - 1))
        continue;
      if (getBan2() && (i == 81 - 1 || i == 82 - 1 || i == 83 - 1))
        continue;
      auto edge = roads[i];
      int v = edge.u;
      if (v == u)
        v = edge.v;
      double w = calcDistance(real_nodes[edge.u], real_nodes[edge.v]);
      if (dist[v] > dist[u] + w) {
        dist[v] = dist[u] + w;
        pre[v] = u;
        heap.push(HeapStruct(v, dist[v] + h(v, t)));
      }
    }
  }
  return 1e9;
}

std::vector<int> getRouteOfWaypoints(int t) {
  std::vector<int> ret;
  for (int u = t; u; u = pre[u]) {
    ret.push_back(u);
  }
  return ret;
}

void initMap() {
  std::map<ReadNode, int> mp_real_nodes;
  readRoads("../SCUTStudentLife/map/real_roads.txt", real_roads, real_nodes,
            mp_real_nodes, mp_node_names);

  int i = 0;
  for (const auto &road : real_roads) {
    int u = mp_real_nodes[road.first], v = mp_real_nodes[road.second];
    if (u > v)
      std::swap(u, v);
    roads.emplace_back(u, v, roads.size()); // now capacity is unlimited,
                                            // and difficulty is all 1
    G[u].push_back(i);
    G[v].push_back(i);
    i++;
  }

  for (auto &real_node : real_nodes) {
    if (real_node0.x > real_node.first && real_node0.y > real_node.second) {
      real_node0.x = real_node.first;
      real_node0.y = real_node.second;
    }
    if (real_node1.x < real_node.first && real_node1.y < real_node.second) {
      real_node1.x = real_node.first;
      real_node1.y = real_node.second;
    }
  }

  pair0 = convert_xy(real_node0.x, real_node0.y);
  pair1 = convert_xy(real_node1.x, real_node1.y);

  int cnt = 0;
  for (auto &real_node : real_nodes) {
    auto temp = convert_node(real_node.first, real_node.second);
    nodes.emplace_back(temp.first, temp.second, cnt++);
  }

  std::vector<ReadEdge> real_walls;
  readWalls("../SCUTStudentLife/map/real_walls.txt", real_walls);

  for (const auto &wall : real_walls) {
    auto temp1 = convert_node(wall.first.first, wall.first.second);
    auto temp2 = convert_node(wall.second.first, wall.second.second);
    walls.emplace_back(std::make_pair(temp1.first, temp1.second),
                       std::make_pair(temp2.first, temp2.second));
    x_max = std::max(x_max, temp1.first);
    x_min = std::min(x_min, temp1.first);
    y_max = std::max(y_max, temp1.second);
    y_min = std::min(y_min, temp1.second);

    x_max = std::max(x_max, temp2.first);
    x_min = std::min(x_min, temp2.first);
    y_max = std::max(y_max, temp2.second);
    y_min = std::min(y_min, temp2.second);
  }

  for (const auto &it : mp_node_names) {
    nodes[it.second].name = it.first;
  }
}

double getMaxX() { return x_max; }
double getMaxY() { return y_max; }
double getMinX() { return x_min; }
double getMinY() { return y_min; }
