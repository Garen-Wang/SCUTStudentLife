#include "social-force.h"
#include "map.h"
#include <QDebug>
#include <vector>

double randomDouble(double lowerBound, double upperBound) {
  return (lowerBound +
          (static_cast<double>(rand()) / RAND_MAX) * (upperBound - lowerBound));
}

std::vector<Agent *> SocialForceModel::getAgents() { return agents; }

std::vector<Wall *> SocialForceModel::getWalls() { return walls; }

void SocialForceModel::addAgent(Agent *agent) { agents.push_back(agent); }

void SocialForceModel::addWall(Wall *wall) { walls.push_back(wall); }

void SocialForceModel::removeAllAgents() {
  for (auto agent : agents) {
    delete agent;
  }
  agents.clear();
}

void SocialForceModel::removeAllWalls() {
  for (auto wall : walls) {
    delete wall;
  }
  walls.clear();
}

void SocialForceModel::move(double time, Time current_time) {
  for (auto agent : agents) {
    agent->move(agents, walls, time, current_time);
  }
}

void SocialForceModel::resetHeap() { waiting_heap = {}; }

void SocialForceModel::initWaitingList(int num_week, int day_of_week,
                                       Time currentTime) {
  qDebug() << "init waiting list";
  for (auto agent : agents) {
    agent->updateWeeklySchedule(num_week);
    auto weekly_schedule = agent->getWeeklySchedule();
    auto event = weekly_schedule->sub_schedules[day_of_week].event_head->next;
    while (event != nullptr && event->trigger_time < currentTime) {
      agent->setCur(mp_node_names[event->target]);
      event = event->next;
    }
    if (event != nullptr)
      waiting_heap.push({agent->getId(), event});
  }
  qDebug() << "now size of waiting_heap:" << waiting_heap.size();
}

void SocialForceModel::checkWaitingList(Time current_time) {
  qDebug() << "check waiting list, size:" << waiting_heap.size();
  while (!waiting_heap.empty()) {
    const auto &elem = waiting_heap.top();
    auto event = elem.event;
    if (event->trigger_time <= current_time) {
      Agent *agent = agents[elem.agent_id];
      qDebug() << elem.agent_id;
      int cur = agent->getCur();
      // allocate an action for an agent to do
      agent->initAction(cur, mp_node_names[event->target],
                        nodes[cur].x + randomDouble(-1.3, 1.3),
                        nodes[cur].y + randomDouble(-2, 2),
                        event->trigger_time);
      waiting_heap.pop(); // pop this event
      event = event->next;
      if (event != nullptr) {
        // push its next event (if any)
        waiting_heap.push({agent->getId(), event});
      }
    } else
      break;
  }
}

SocialForceModel::SocialForceModel() : agents(), walls(), waiting_heap() {}
SocialForceModel::~SocialForceModel() {
  for (auto agent : agents) {
    delete agent;
  }
  agents.clear();

  for (auto wall : walls) {
    delete wall;
  }
  walls.clear();
}
