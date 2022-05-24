#include "agent.h"

#include "config.h"
#include "map.h"
#include "vecmath.h"
#include <QDebug>
#include <cassert>
#include <cmath>
#include <random>

std::default_random_engine gen;

static int id_cnt = 0;

Agent::Agent(std::shared_ptr<StudentInfo> student_info, std::string dorm_name)
    : id(id_cnt++), num_week(-1), start(-1), cur(-1), destination(-1),
      reverse(false), road(-1), finished(true), start_time(), radius(0.2),
      pos(0, 0, 0), velocity(0, 0, 0), group(nullptr),
      student_info(student_info), weekly_schedule(nullptr),
      dorm_name(dorm_name), transport(OnFoot) {
  qDebug() << "dorm_name: " << QString::fromStdString(dorm_name);
  assert(mp_node_names.count(dorm_name) != 0);
  cur = mp_node_names[dorm_name];
  assert(transport == OnFoot);
  std::normal_distribution<double> normal(1.29, 0.19);
  desired_speed = normal(gen);
}

Agent::Agent(std::shared_ptr<StudentInfo> student_info, std::string dorm_name,
             AgentTransport transport)
    : Agent(student_info, dorm_name) {
  this->transport = transport;
  if (transport == Bicycle) {
    std::normal_distribution<double> normal(2.38, 0.38);
    desired_speed = normal(gen);
  } else if (transport == ElectricBicycle) {
    std::normal_distribution<double> normal(3.1, 0.38);
    desired_speed = normal(gen);
  } else {
    std::normal_distribution<double> normal(1.29, 0.19);
    desired_speed = normal(gen);
  }
}

Agent::Agent(std::shared_ptr<StudentInfo> student_info, std::string dorm_name,
             std::shared_ptr<Group> group)
    : Agent(student_info, dorm_name) {
  this->group = group;
  group->addMember(this);
}

Agent::Agent(std::shared_ptr<StudentInfo> student_info, std::string dorm_name,
             std::shared_ptr<Group> group, AgentTransport transport)
    : Agent(student_info, dorm_name, transport) {
  this->group = group;
  group->addMember(this);
  this->transport = transport;
}

Agent::~Agent() {
  for (auto daily_schedule : weekly_schedule->sub_schedules) {
    std::vector<Event *> ptrs;
    for (auto p = daily_schedule.event_head; p != nullptr; p++)
      ptrs.push_back(p);
    for (auto it = ptrs.rbegin(); it != ptrs.rend(); ++it) {
      delete *it;
    }
    ptrs.clear();
  }
  delete weekly_schedule;
}

Vector3d Agent::getTargetMotivationForce() {
  if (finished)
    return Vector3d(0, 0, 0);
  //  Waypoint waypoint = getNextWaypoint();
  //  const double TAU = 0.54;
  //  Vector3d desired_direction = waypoint.pos - pos;
  const double TAU = 0.54;
  Vector3d desired_direction;
  if (!group)
    desired_direction = getNextWaypoint().pos - pos;
  else
    desired_direction = group->getNextWaypoint().pos - pos;
  desired_direction.normalize();
  Vector3d desired_velocity = desired_speed * desired_direction;
  Vector3d diff_velocity = desired_velocity - velocity;
  Vector3d f_i_0 = diff_velocity * (1 / TAU); // no division, use reciprocals
  return f_i_0;
}

Vector3d Agent::getWallInteractionForce(const std::vector<Wall *> &walls) {
  if (finished)
    return Vector3d(0, 0, 0);
  const double a = 10, b = 0.1;
  Vector3d res_diff_vector;
  double res_sq_length = 1e18;
  for (auto wall : walls) {
    Point3d nearest_point = wall->getNearestPoint(pos);
    Vector3d diff_vector = pos - nearest_point; // direction: from wall to agent
    double sq_length = diff_vector.lengthSquared();
    if (sq_length < res_sq_length) {
      res_sq_length = sq_length;
      res_diff_vector = diff_vector;
    }
  }
  double d_w = sqrt(res_sq_length) - radius;
  double f_i_wall = a * exp(-d_w / b);
  res_diff_vector.normalize();
  return f_i_wall * res_diff_vector;
}

Vector3d Agent::getAgentInteractionForce(const std::vector<Agent *> &agents) {
  if (finished)
    return Vector3d(0, 0, 0);

  const double A = 4.5;       // pm 0.3
  const double gamma = 0.35;  // pm 0.01
  const double n = 2.0;       // pm 0.1
  const double n_prime = 3.0; // pm 0.7
  const double lambda = 2.0;  // pm 0.2

  Vector3d ret(0, 0, 0);
  auto size = agents.size();
  int cnt = 0;
  for (auto agent : agents) {
    cnt++;
    if (agent->id == this->id)
      continue;
    if (agent->isFinished())
      continue;
    Vector3d x_i_j = agent->pos - this->pos;
    if (x_i_j.lengthSquared() > 2 * 2)
      continue;

    Vector3d e_i_j = x_i_j;
    e_i_j.normalize();

    Vector3d v_i_j = this->velocity - agent->velocity; // fixed
    Vector3d D_i_j = lambda * v_i_j + e_i_j;
    Vector3d t_i_j = D_i_j;
    t_i_j.normalize();
    Vector3d n_i_j(-t_i_j.y, t_i_j.x, 0);

    double B_i_j = gamma * D_i_j.length();

    double theta_i_j = t_i_j.angle(e_i_j);

    // double K = theta_i_j == 0.0 ? 0.0 : theta_i_j / fabs(theta_i_j);
    double K = (theta_i_j < 1e-14) ? 0.0 : theta_i_j / fabs(theta_i_j);

    double d_i_j = x_i_j.length();

    double f_v = -A * exp(-d_i_j / B_i_j - (n_prime * B_i_j * theta_i_j) *
                                               (n_prime * B_i_j * theta_i_j));
    double f_theta =
        -A * K *
        exp(-d_i_j / B_i_j - (n * B_i_j * theta_i_j) * (n * B_i_j * theta_i_j));

    Vector3d temp = f_v * t_i_j + f_theta * n_i_j;
    ret += temp;
  }
  return ret;
}

void Agent::setPos(double x, double y) { pos.set(x, y, 0); }

void Agent::setStart(int start) { this->start = start; }

void Agent::setCur(int cur) { this->cur = cur; }

void Agent::setDestination(int destination) {
  this->destination = destination;
  if (!group)
    updateWaypoints(cur, destination);
  else
    group->updateWaypoints(cur, destination);
}

Point3d Agent::getPos() { return pos; }
double Agent::getX() { return pos.x; }
double Agent::getY() { return pos.y; }

// void Agent::setWaypoint(int u, double x, double y, double radius) {
//   waypoints.push_back(Waypoint(u, x, y, radius));
// }

Waypoint Agent::getNextWaypoint() {
  if (group) {
    qDebug() << group->waypoints.size();
    return group->getNextWaypoint();
  }
  assert(!waypoints.empty()); // guarantee that the deque waypoints is not empty
  return waypoints.front();
}

void Agent::move(const std::vector<Agent *> &agents,
                 const std::vector<Wall *> &walls, double time,
                 Time current_time) {
  if ((!group && waypoints.empty()) && !finished) {
    finished = true;
    // debug begin
    std::cout << "agent " << id << ", from " << start_time << " to "
              << current_time << ", from ";
    if (nodes[start].name.length() > 0) {
      std::cout << nodes[start].name;
    }
    std::cout << " to ";
    if (nodes[destination].name.length() > 0) {
      std::cout << nodes[destination].name;
    } else
      std::cout << destination;
    std::cout << std::endl;
    // debug end
    start = destination = road = -1;
  }
  if (finished)
    return;

  auto v = getNextWaypoint().u;
  bool reverse = false;
  int road_idx = -1;
  for (int i : G[getCur()]) {
    if (getBan1() && (i >= 60 - 1 && i <= 62 - 1))
      continue;
    if (getBan2() && (i == 81 - 1 || i == 82 - 1 || i == 85 - 1))
      continue;
    auto road = roads[i];
    if (road.u != v && road.v != v)
      continue;
    if (road.u == getCur())
      reverse = false;
    else
      reverse = true;
    road_idx = i;
  }
  assert(road_idx != -1);
  if (road != road_idx) {
    roads[road].num_agents--;
    road = road_idx;
    roads[road].num_agents++;
  }

  Vector3d target_motivation_force = getTargetMotivationForce();
  Vector3d agent_interation_force = getAgentInteractionForce(agents);
  Vector3d wall_interaction_force = getWallInteractionForce(walls);
  Vector3d classic_acceleration =
      target_motivation_force + agent_interation_force + wall_interaction_force;

  Vector3d acceleration = classic_acceleration;
  if (group != nullptr) {
    Vector3d group_vision_force = getGroupVisionForce();
    Vector3d group_attractive_force = getGroupAttractiveForce();
    Vector3d group_repulsive_force = getGroupRepulsiveForce();
    Vector3d group_acceleration =
        group_vision_force + group_attractive_force + group_repulsive_force;
    acceleration += group_acceleration;
  }

  velocity = velocity + acceleration * time;
  double difficulty = roads[road_idx].difficulty;

  double ratio = 1.0 * roads[road_idx].num_agents / roads[road_idx].capacity;
  if (ratio > 0.7) {
    // a linearly decreasing function
    double temp = -7.0 / 3.0 * ratio + 79.0 / 30.0;
    assert(temp > 0);
    difficulty *= temp;
  }
  assert(difficulty > 0 && difficulty <= 1);

  if (velocity.length() > desired_speed * difficulty) {
    velocity.normalize();
    velocity = velocity * desired_speed * difficulty;
  }

  pos = pos + velocity * time;

  // update the current status with waypoints
  std::deque<Waypoint> &wps = waypoints;
  if (group)
    wps = group->waypoints;
  assert(wps[0].u != getCur()); // cannot go to the current place
  Vector3d current_dist = wps[0].pos - pos;

  if (wps.size() > 2) {
    Vector3d next_dist = wps[1].pos - pos;
    if (next_dist.lengthSquared() < current_dist.lengthSquared()) {
      // wps.push_back(wps.front());
      setCur(wps.front().u);
      wps.pop_front();
      current_dist = next_dist;
    }
  }

  double r = wps.front().radius;
  if (current_dist.lengthSquared() < r * r) {
    // wps.push_back(wps.front());
    auto x = wps.front().u;
    setCur(x);
    if (!wps.empty())
      wps.pop_front();
  }
}

void Agent::setFinished(bool x) { finished = x; }

void Agent::initAction(int s, int t, double x, double y, Time start_time) {
  setStart(s);
  setStartTime(start_time);
  setCur(s);
  setPos(x, y);
  setDestination(t);
  setFinished(false); // go!
}

void Agent::updateWaypoints(int u, int t) {
  // aStar(cur, t);
  bfs(this, u, t);
  waypoints.clear();

  auto wps = getRouteOfWaypoints(t);
  for (int wp : wps) {
    // radius is flexible here actually
    waypoints.push_front(Waypoint(wp, nodes[wp].x, nodes[wp].y, 2));
  }
  waypoints.pop_front();
}

double Agent::getRadius() const { return radius; }

double Agent::getDesiredSpeed() const { return desired_speed; }

AgentTransport Agent::getTransport() const { return transport; }

std::string Agent::getDormName() const { return dorm_name; }

std::shared_ptr<StudentInfo> Agent::getStudentInfo() const {
  return student_info;
}

Vector3d Agent::getGroupVisionForce() {
  if (group == nullptr || group->size() <= 1 || finished)
    return {0, 0, 0};

  double beta1 = 0.5;       // mutable parameter
  double vision_angle = 90; // mutable parameter
  size_t size = group->size();
  if (size <= 1)
    return {0, 0, 0};

  Vector3d members_com = group->getOtherCenterOfMass(this);
  members_com.normalize();

  Vector3d direction = getDesiredDirection();
  direction.normalize();
  double cos_theta = members_com.dot(direction);
  double theta = acos(cos_theta);
  theta = theta / PI * 180;
  double rotation = std::max(theta - vision_angle, 0.0);
  rotation = rotation / 180 * PI;
  Vector3d force_vector = -rotation * direction;
  return beta1 * force_vector;
}

Vector3d Agent::getGroupAttractiveForce() {
  if (group == nullptr || group->size() <= 1 || finished)
    return Vector3d(0, 0, 0);
  double beta2 = 1;                                   // mutable parameter
  double threshold = (group->size() - 1) * 0.5 * 0.5; // mutable parameter
  Vector3d force_vector = group->getTotalCenterOfMass() - pos;
  double sq_norm = force_vector.lengthSquared();
  force_vector.normalize();
  if (sq_norm <= threshold * threshold) {
    return Vector3d(0, 0, 0);
  }
  return beta2 * force_vector;
}

Vector3d Agent::getGroupRepulsiveForce() {
  if (group == nullptr || group->size() <= 1 || finished)
    return Vector3d(0, 0, 0);
  double threshold = 0.5;
  double beta3 = 1; // mutable parameter

  Vector3d ans_vector(0, 0, 0);
  for (Agent *member : group->getMembers()) {
    if (member->getId() == id)
      continue;
    Vector3d force_vector = member->getPos() - pos;
    double sq_norm = force_vector.lengthSquared();
    force_vector.normalize();
    if (sq_norm <= threshold * threshold) {
      ans_vector += beta3 * force_vector;
    }
  }
  return ans_vector;
}

Vector3d Agent::getDesiredDirection() {
  //  Waypoint waypoint = getNextWaypoint();
  //  Vector3d desired_direction = waypoint.pos - pos;
  //  desired_direction.normalize();
  //  return desired_direction;
  Vector3d desired_direction;
  if (!group)
    desired_direction = getNextWaypoint().pos - pos;
  else
    desired_direction = group->getNextWaypoint().pos - pos;
  desired_direction.normalize();
  return desired_direction;
}
int Agent::getId() { return id; }
void Agent::setStartTime(Time start_time) { this->start_time = start_time; }
bool Agent::isFinished() { return finished; }

bool Agent::hasGroup() { return group != nullptr; }

void Agent::setWeeklySchedule(WeeklySchedule *weekly_schedule) {
  this->weekly_schedule = weekly_schedule;
}
void Agent::updateWeeklySchedule(int num_week) {
  if (this->num_week != num_week) {
    auto weekly_schedule =
        student_info->generateWeeklySchedule(num_week, dorm_name);
    setWeeklySchedule(weekly_schedule);
    this->num_week = num_week;
  }
}

WeeklySchedule *Agent::getWeeklySchedule() const { return weekly_schedule; }

int Agent::getCur() const {
  if (!group)
    return cur;
  else
    return group->getMembers()[0]->cur;
}

// group

Group::Group() : members(), waypoints() {}
Group::Group(const std::vector<Agent *> &members) : members(members) {}

void Group::addMember(Agent *member) { members.push_back(member); }

std::vector<Agent *> Group::getMembers() { return members; }

size_t Group::size() { return members.size(); }
Point3d Group::getTotalCenterOfMass() {
  Point3d com(0, 0, 0);
  for (auto member : members) {
    com += member->getPos();
  }
  com *= 1.0 / members.size();
  return com;
}
Point3d Group::getOtherCenterOfMass(Agent *agent) {
  Point3d com(0, 0, 0);
  for (auto member : members) {
    if (member->getId() == agent->getId())
      continue;
    com += member->getPos();
  }
  assert(members.size() >= 2);
  com *= 1.0 / (members.size() - 1);
  return com;
}

void Group::updateWaypoints(int u, int t) {
  // aStar(cur, t);
  bfs(getMembers()[0], u, t);
  waypoints.clear();

  auto wps = getRouteOfWaypoints(t);
  for (int wp : wps) {
    waypoints.push_front(Waypoint(wp, nodes[wp].x, nodes[wp].y, 2));
  }
  waypoints.pop_front();
}

Waypoint Group::getNextWaypoint() {
  assert(!waypoints.empty()); // guarantee that the deque waypoints is not empty
  return waypoints.front();
}
