#include "loader.h"
#include <QDebug>
#include <QDir>
#include <fstream>

AgentLoader::AgentLoader(SocialForceModel *socialForce)
    : socialForce(socialForce) {}

AgentTransport randomChoice() {
  int x = rand() % 100;
  if (x < 70)
    return OnFoot;
  else if (x < 95)
    return Bicycle;
  else
    return ElectricBicycle;
}

void AgentLoader::generateFromJsonFile(std::string filename, int numAgent,
                                       std::string majorDormName,
                                       std::string minorDormName) {
  std::ifstream ifs("../SCUTStudentLife/schedule/json/" + filename);
  assert(ifs.is_open());
  json j;
  ifs >> j;
  ifs.close();
  auto studentInfo = std::make_shared<StudentInfo>(filename);
  std::transform(majorDormName.begin(), majorDormName.end(),
                 majorDormName.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  std::transform(minorDormName.begin(), minorDormName.end(),
                 minorDormName.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  for (int i = 0; i < numAgent * 4 / 5; i++) {
    auto agent = new Agent(studentInfo, majorDormName, randomChoice());
    socialForce->addAgent(agent);
  }
  for (int i = 0; i < numAgent / 5; i++) {
    auto agent = new Agent(studentInfo, minorDormName, randomChoice());
    socialForce->addAgent(agent);
  }
}

void AgentLoader::generateAll() {
  QDir jsonDir("../SCUTStudentLife/schedule/json/");
  jsonDir.setFilter(QDir::Files);
  QStringList entries = jsonDir.entryList();
  for (auto entry = entries.begin(); entry != entries.end(); entry++) {
    qDebug() << "reading json file:" << *entry;
    int dorm1 = rand() % 15 + 1;
    int dorm2 = rand() % 15 + 1;
    while (dorm1 == dorm2) {
      dorm2 = rand() % 15 + 1;
    }
    generateFromJsonFile(entry->toStdString(), 50, "c" + std::to_string(dorm1),
                         "c" + std::to_string(dorm2));
  }
}
