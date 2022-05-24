#ifndef DSPROJECT_READER_H
#define DSPROJECT_READER_H

#include "config.h"
#include "map.h"
#include <string>
#include <vector>

void readRoads(const std::string &filename, std::vector<ReadEdge> &roads,
               std::vector<ReadNode> &real_nodes,
               std::map<ReadNode, int> &mp_real_nodes,
               std::map<std::string, int> &mp_names_of_nodes);

void readWalls(const std::string &filename, std::vector<ReadEdge> &vec_walls);

#endif // DSPROJECT_READER_H
