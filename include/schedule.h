#ifndef DSPROJECT_SCHEDULE_H
#define DSPROJECT_SCHEDULE_H

#include <iostream>
#include <string>
#include <vector>

#include "config.h"

enum EventName {
  Run,          // no need for a target, just wandering for minutes
  AttendClass,  // target is a classroom
  EatInCanteen, // target is a canteen
  BackToDorm,   // target is a dorm building
};

class Time {
public:
  int h;  // [0, 24]
  int m;  // [0, 60]
  int s;  // [0, 60]
  int ms; // [0, 1000]

  explicit Time();
  explicit Time(int h, int m);
  explicit Time(int h, int m, int s);
  explicit Time(int h, int m, int s, int ms);
  bool operator==(const Time &rhs) const;
  bool operator<(const Time &rhs) const;
  bool operator<=(const Time &rhs) const;
  bool operator>(const Time &rhs) const;
  bool operator>=(const Time &rhs) const;
  friend std::ostream &operator<<(std::ostream &out, const Time &time);
  Time &operator+=(double frameTime);
  void advance(int h, int m, int s);
  void advance(Time duration);
  void addSec(int s);
  void set(int h, int m, int s);
};

// event organized in linked-list
class Event {
public:
  Time trigger_time;
  EventName event_name;
  std::string target;
  Event *next;
  explicit Event(){};
  explicit Event(Time trigger_time, EventName event_name, std::string target)
      : trigger_time(trigger_time), event_name(event_name), target(target),
        next(nullptr) {}
};

// schedule during a day
class DailySchedule {
public:
  int day_of_week;
  std::string last_name;

  // events are organized as a singly linked-list
  Event *event_head, *event_tail;
  explicit DailySchedule(int day_of_week, std::string dorm_name)
      : day_of_week(day_of_week), last_name(dorm_name), event_head(new Event()),
        event_tail(event_head) {}
  //  ~DailySchedule();
  bool insert(Time time, EventName event_name, std::string dest_name);
  void clear();
};

// schedule during a week
class WeeklySchedule {
public:
  std::vector<DailySchedule> sub_schedules;
  explicit WeeklySchedule();
  bool insert(DailySchedule daily_schedule);
  bool empty() const;
};

class Course {
public:
  std::string name;
  int day_of_week; // [0, 6]
  int start_period;
  int end_period;
  std::string classroom;

  explicit Course(std::string name, int day_of_week, int start_period,
                  int end_period, std::string classroom)
      : name(name), day_of_week(day_of_week), start_period(start_period),
        end_period(end_period), classroom(classroom) {}
};

class WeeklyCourses {
public:
  std::vector<Course> courses;
};

class StudentInfo {
public:
  std::string class_name;
  std::string filename;
  json j;

  explicit StudentInfo(std::string filename);
  WeeklySchedule *generateWeeklySchedule(int num_week, std::string dorm_name);
};

#endif // DSPROJECT_SCHEDULE_H
