#ifndef DSPROJECT_SOCIAL_FORCE_H
#define DSPROJECT_SOCIAL_FORCE_H

#include <queue>
#include <vector>

#include "agent.h"
#include "wall.h"

class SocialForceModel {
private:
  std::vector<Agent *> agents;
  std::vector<Wall *> walls;

  struct WaitingHeapElem {
    int agent_id;
    Event *event;

    bool operator<(const WaitingHeapElem &rhs) const {
      // min heap
      if (event->trigger_time == rhs.event->trigger_time)
        return agent_id > rhs.agent_id;
      return event->trigger_time > rhs.event->trigger_time;
    }
  };

  std::priority_queue<WaitingHeapElem> waiting_heap;

public:
  explicit SocialForceModel();
  ~SocialForceModel();
  std::vector<Agent *> getAgents();
  std::vector<Wall *> getWalls();

  void addAgent(Agent *agent);
  void addWall(Wall *wall);

  void removeAllAgents();
  void removeAllWalls();

  void initWaitingList(int num_week, int day_of_week, Time currentTime);
  void checkWaitingList(Time current_time);

  void move(double time, Time current_time);
  void resetHeap();
};

#endif // DSPROJECT_SOCIAL_FORCE_H
