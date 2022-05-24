#ifndef DSPROJECT_AGENT_H
#define DSPROJECT_AGENT_H

#include "schedule.h"
#include "vecmath.h"
#include "wall.h"
#include <deque>
#include <memory>
#include <vector>

struct Waypoint {
  Point3d pos;
  int u;
  double radius;
  explicit Waypoint(int u, double x, double y, double radius)
      : u(u), pos(x, y, 0), radius(radius) {}
};

class Agent;

class Group {
private:
  std::vector<Agent *> members;
  int cur;

public:
  explicit Group();
  explicit Group(const std::vector<Agent *> &members);
  void addMember(Agent *member);
  std::vector<Agent *> getMembers();
  Point3d getTotalCenterOfMass();
  Point3d getOtherCenterOfMass(Agent *agent);
  size_t size();

  std::deque<Waypoint> waypoints;
  Waypoint getNextWaypoint();
  void updateWaypoints(int u, int t);
};

enum AgentTransport { OnFoot, Bicycle, ElectricBicycle };

class Agent {
private:
  AgentTransport transport;
  int id;       // agent id
  int num_week; // current week

  int start;       // start idx
  int cur;         // current idx
  int destination; // destination idx

  bool reverse;    // whether moving in reverse or not
  int road;        // road idx
  bool finished;   // move only when true
  Time start_time; // start time

  Point3d pos;                  // current position
  std::shared_ptr<Group> group; // notnull when moving in group
  Vector3d velocity;            // velocity
  double radius;
  double desired_speed;
  std::deque<Waypoint> waypoints;
  std::shared_ptr<StudentInfo> student_info;
  WeeklySchedule *weekly_schedule;
  std::string dorm_name;

public:
  explicit Agent(std::shared_ptr<StudentInfo> student_info,
                 std::string dorm_name);
  explicit Agent(std::shared_ptr<StudentInfo> student_info,
                 std::string dorm_name, AgentTransport transport);
  explicit Agent(std::shared_ptr<StudentInfo> student_info,
                 std::string dorm_name, std::shared_ptr<Group> group);
  explicit Agent(std::shared_ptr<StudentInfo> student_info,
                 std::string dorm_name, std::shared_ptr<Group> group,
                 AgentTransport transport);
  ~Agent();
  Vector3d getTargetMotivationForce();
  Vector3d getWallInteractionForce(const std::vector<Wall *> &walls);
  Vector3d getAgentInteractionForce(const std::vector<Agent *> &agents);
  Vector3d getGroupVisionForce();
  Vector3d getGroupAttractiveForce();
  Vector3d getGroupRepulsiveForce();

  void setPos(double x, double y);
  void setStart(int start);
  void setCur(int cur);
  void setDestination(int destination);
  void setStartTime(Time start_time);
  //  void setWaypoint(int u, double x, double y, double radius);
  void setFinished(bool x);

  int getId();
  Point3d getPos();
  double getX();
  double getY();
  double getRadius() const;
  double getDesiredSpeed() const;
  AgentTransport getTransport() const;
  int getCur() const;
  std::string getDormName() const;
  std::shared_ptr<StudentInfo> getStudentInfo() const;
  Vector3d getDesiredDirection();
  Waypoint getNextWaypoint();
  WeeklySchedule *getWeeklySchedule() const;

  void initAction(int s, int t, double x, double y, Time start_time);
  void move(const std::vector<Agent *> &agents,
            const std::vector<Wall *> &walls, double time, Time current_time);
  void updateWaypoints(int u, int t);
  void updateWeeklySchedule(int num_week);
  bool isFinished();
  bool hasGroup();

protected:
  void setWeeklySchedule(WeeklySchedule *weekly_schedule);
};

#endif // DSPROJECT_AGENT_H
