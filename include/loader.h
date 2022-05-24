#ifndef LOADER_H
#define LOADER_H

#include <QString>

#include "social-force.h"

class AgentLoader {
public:
  SocialForceModel *socialForce;

  explicit AgentLoader(SocialForceModel *socialForce);

  void generateFromJsonFile(std::string filename, int numAgent,
                            std::string majorDormName,
                            std::string minorDormName);
  void generateAll();
};

#endif // LOADER_H
