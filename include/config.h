#ifndef DSPROJECT_CONFIG_H
#define DSPROJECT_CONFIG_H

#include "json.hpp"
#include <utility>

// Global Constant Variables
const double PI = 3.14159265359;
const double R = 6371; // in kilometer

typedef std::pair<double, double> ReadNode;
typedef std::pair<ReadNode, ReadNode> ReadEdge;

using json = nlohmann::json;

#endif // DSPROJECT_CONFIG_H
