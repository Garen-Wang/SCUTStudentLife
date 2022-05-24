#include "reader.h"
#include "map.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>

// read roads (graph of abstract nodes) for pathfinding from *filename*
void readRoads(const std::string &filename, std::vector<ReadEdge> &roads,
               std::vector<ReadNode> &real_nodes,
               std::map<ReadNode, int> &mp_real_nodes,
               std::map<std::string, int> &mp_names_of_nodes) {
  const std::regex reg(
      R"(^\((.*?), *(.*?)\), *\((.*?), *(.*?)\)(, *\[(.*?), *(.*?)\])? *(//.*?)?$)");
  std::ifstream ifs(filename);
  assert(ifs.is_open());
  std::smatch matches;
  std::string line;

  int cnt = 0;
  while (std::getline(ifs, line)) {
    if (std::regex_search(line, matches, reg)) {
      assert(matches.size() == 8 + 1);
      double x1 = std::stod(matches[1]);
      double y1 = std::stod(matches[2]);
      double x2 = std::stod(matches[3]);
      double y2 = std::stod(matches[4]);
      roads.push_back(
          std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
      auto p1 = std::make_pair(x1, y1);
      if (mp_real_nodes.find(p1) == mp_real_nodes.end()) {
        mp_real_nodes[p1] = cnt++;
        real_nodes.push_back(p1);
      }
      auto p2 = std::make_pair(x2, y2);
      if (mp_real_nodes.find(p2) == mp_real_nodes.end()) {
        mp_real_nodes[p2] = cnt++;
        real_nodes.push_back(p2);
      }
      if (matches[6].length() > 0) {
        if (mp_names_of_nodes.find(matches[6]) != mp_names_of_nodes.end()) {
          int idx1 = mp_real_nodes[std::make_pair(x1, y1)];
          int idx2 = mp_names_of_nodes[matches[6]];
          assert(idx1 == idx2);
        } else {
          mp_names_of_nodes[matches[6]] = mp_real_nodes[std::make_pair(x1, y1)];
        }
      }
      if (matches[7].length() > 0) {
        if (mp_names_of_nodes.find(matches[7]) != mp_names_of_nodes.end()) {
          int idx1 = mp_real_nodes[std::make_pair(x2, y2)];
          int idx2 = mp_names_of_nodes[matches[7]];
          assert(idx1 == idx2);
        } else {
          mp_names_of_nodes[matches[7]] = mp_real_nodes[std::make_pair(x2, y2)];
        }
      }
    }
  }
}

// read walls (2D line segments) from *filename*
void readWalls(const std::string &filename, std::vector<ReadEdge> &vec_walls) {
  const std::regex reg(R"(\((.*?), ?(.*?)\), ?\((.*?), ?(.*?)\))");
  std::ifstream ifs(filename);
  assert(ifs.is_open());
  std::smatch matches;
  std::string line;

  while (std::getline(ifs, line)) {
    if (std::regex_search(line, matches, reg)) {
      assert(matches.size() == 4 + 1);
      double x1 = std::stod(matches[1]);
      double y1 = std::stod(matches[2]);
      double x2 = std::stod(matches[3]);
      double y2 = std::stod(matches[4]);
      vec_walls.emplace_back(std::make_pair(x1, y1), std::make_pair(x2, y2));
    }
  }
}
