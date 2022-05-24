#include <QDebug>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>

#include "config.h"
#include "schedule.h"

Time::Time() : h(0), m(0), s(0), ms(0) {}
Time::Time(int h, int m) : h(h), m(m) {}
Time::Time(int h, int m, int s) : h(h), m(m), s(s), ms(0) {}
Time::Time(int h, int m, int s, int ms) : h(h), m(m), s(s), ms(ms) {}
bool Time::operator<(const Time &rhs) const {
  if (h == rhs.h) {
    if (m == rhs.m) {
      if (s == rhs.s)
        return ms < rhs.ms;
      else
        return s < rhs.s;
    } else
      return m < rhs.m;
  } else
    return h < rhs.h;
}
bool Time::operator>=(const Time &rhs) const { return !((*this) < rhs); }
std::ostream &operator<<(std::ostream &out, const Time &time) {
  out << time.h << ":" << time.m << ":" << time.s;
  return out;
}

Time &Time::operator+=(double frameTime) {
  ms += static_cast<int>(frameTime);
  if (ms >= 1000) {
    s += ms / 1000;
    ms %= 1000;
  }
  if (s >= 60) {
    m += s / 60;
    s %= 60;
  }
  if (m >= 60) {
    h += m / 60;
    m %= 60;
  }
  assert(h < 24);
  return *this;
}
void Time::advance(Time duration) {
  advance(duration.h, duration.m, duration.s);
}
void Time::advance(int h, int m, int s) {
  this->s += s;
  if (this->s >= 60) {
    this->m += this->s / 60;
    this->s %= 60;
  }
  this->m += m;
  if (this->m >= 60) {
    this->h += m / 60;
    this->m %= 60;
  }
  this->h += h;
  assert(h < 24);
}
void Time::addSec(int s) {
  this->s += s;
  if (this->s >= 60) {
    this->m += this->s / 60;
    this->s %= 60;
  }
  if (this->m >= 60) {
    this->h += m / 60;
    this->m %= 60;
  }
  assert(h < 24);
}
void Time::set(int h, int m, int s) {
  this->h = h;
  this->m = m;
  this->s = s;
  this->ms = 0;
}

bool Time::operator<=(const Time &rhs) const {
  return (*this) < rhs || (*this) == rhs;
}

bool Time::operator==(const Time &rhs) const {
  return h == rhs.h && m == rhs.m && s == rhs.s && ms == rhs.ms;
}

bool Time::operator>(const Time &rhs) const {
  if (h == rhs.h) {
    if (m == rhs.m) {
      if (s == rhs.s)
        return ms > rhs.ms;
      else
        return s > rhs.s;
    } else
      return m > rhs.m;
  } else
    return h > rhs.h;
}

bool DailySchedule::insert(Time time, EventName event_name,
                           std::string dest_name) {
  if (dest_name != last_name) {
    // qDebug() << "time:" << time.h << time.m << time.s;
    event_tail->next = new Event(time, event_name, dest_name);
    event_tail = event_tail->next;
    last_name = dest_name;
  }
  return true;
}

WeeklySchedule::WeeklySchedule() : sub_schedules() {}

bool WeeklySchedule::insert(DailySchedule daily_schedule) {
  sub_schedules.push_back(daily_schedule);
  return true;
}

bool WeeklySchedule::empty() const { return sub_schedules.empty(); };

std::string randomChoice(std::string choice1, std::string choice2) {
  return rand() & 2 ? choice1 : choice2;
}

WeeklyCourses generateCoursesFromTable(json j, int num_week) {
  WeeklyCourses weekly_courses;
  for (auto &elem : j) {
    std::string classroom = elem["classroom"];
    if (classroom.length() > 2) {
      classroom = classroom.substr(0, 2);
    }
    // transform classroom name to lowercase
    std::transform(classroom.begin(), classroom.end(), classroom.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (classroom[0] == 'a' && classroom[1] >= '1' && classroom[1] <= '4') {
      // classroom in [a1, a2, a3, a4]
      for (auto &week_interval : elem["week"]) {
        if (week_interval.size() == 1 && num_week == week_interval ||
            week_interval.size() == 2 && num_week >= week_interval[0] &&
                num_week <= week_interval[1]) {
          for (auto &period : elem["period"]) {
            int day_of_week = period[0];
            int start_period = period[1];
            int end_period = period[2];

            weekly_courses.courses.emplace_back(
                elem["name"], day_of_week, start_period, end_period, classroom);
          }
        }
      }
    }
  }
  return weekly_courses;
}

StudentInfo::StudentInfo(std::string filename) : filename(filename) {
  std::ifstream ifs("../SCUTStudentLife/schedule/json/" + filename);
  class_name = filename.substr(0, filename.length() - 7); // trim "课表.json";
  qDebug() << QString::fromStdString(class_name);
  assert(ifs.is_open());
  ifs >> j; // read json file to *j*
  ifs.close();
}

Time randomTimeInterval(Time start_time, Time end_time) {
  int start = start_time.h * 3600 + start_time.m * 60 + start_time.s;
  int end = end_time.h * 3600 + end_time.m * 60 + end_time.s;
  int diff = end - start;
  int x = rand() % diff;
  start_time.addSec(x);
  return start_time;
}

WeeklySchedule *StudentInfo::generateWeeklySchedule(int num_week,
                                                    std::string dorm_name) {
  std::map<int, Time> period_over_time;
  period_over_time[1] = Time(9, 35, 0);
  period_over_time[2] = Time(10, 25, 0);
  period_over_time[3] = Time(11, 25, 0);
  period_over_time[4] = Time(12, 10, 0);
  period_over_time[5] = Time(2 + 12, 45, 0);
  period_over_time[6] = Time(3 + 12, 35, 0);
  period_over_time[7] = Time(4 + 12, 30, 0);
  period_over_time[8] = Time(5 + 12, 15, 0);
  period_over_time[9] = Time(7 + 12, 45, 0);
  period_over_time[10] = Time(8 + 12, 35, 0);
  period_over_time[11] = Time(9 + 12, 25, 0);

  if (j.empty()) {
    std::ifstream ifs("../SCUTStudentLife/schedule/json/" + filename);
    assert(ifs.is_open());
    ifs >> j;
    ifs.close();
  }
  auto weekly_courses = generateCoursesFromTable(j, num_week);

  //  std::string dorm_name = "c12";
  auto weekly_schedule = new WeeklySchedule();

  for (int i = 0; i < 7; i++) {
    DailySchedule daily_schedule(i, dorm_name);
    // day_of_week is *i*, from 0 to 6 (inclusive)
    std::vector<Course> daily_courses;

    for (const auto &course : weekly_courses.courses) {
      if (course.day_of_week == i) {
        daily_courses.push_back(course);
      }
    }
    std::sort(daily_courses.begin(), daily_courses.end(),
              [](const Course &lhs, const Course &rhs) {
                return lhs.start_period < rhs.start_period;
              });

    // go out from dormitory at a random time moment from 7:50 to 8:10
    Time time = randomTimeInterval(Time(7, 50, 0), Time(8, 10, 0));
    // the destination is either canteen1 or canteen2 (for breakfast)
    daily_schedule.insert(time, EatInCanteen,
                          randomChoice("canteen1", "canteen2"));

    // assume that it takes 5 to 20 minutes to have breakfast
    time.advance(randomTimeInterval(Time(0, 5, 0), Time(0, 20, 0)));

    auto it = daily_courses.begin();
    // checking whether a student has lectures in the morning
    int morning = 0;
    while (it != daily_courses.end()) {
      if (it->start_period > 4)
        break;
      // go to attend the lecture
      daily_schedule.insert(time, AttendClass, it->classroom);

      // avoid setting all students rushing out suddenly
      // set up a time range of 5mins
      auto time_end = period_over_time[it->end_period];
      time_end.advance(0, 5, 0);
      time = randomTimeInterval(period_over_time[it->end_period], time_end);

      morning++;
      it++;
    }
    // special case: no morning lectures
    if (morning == 0) {
      // go back to dorm after having breakfast
      daily_schedule.insert(time, BackToDorm, dorm_name);

      // go for lunch at a random time moment from 11:30 to 12:30
      time = randomTimeInterval(Time(11, 30, 0), Time(12, 30, 0));
    }
    daily_schedule.insert(time, EatInCanteen,
                          randomChoice("canteen1", "canteen2"));

    // assume that it takes 10 to 30 minutes to have lunch
    time.advance(randomTimeInterval(Time(0, 10, 0), Time(0, 30, 0)));
    // after having lunch, go back to dorm for a lunch break
    daily_schedule.insert(time, BackToDorm, dorm_name);

    // checking whether a student has lectures in the afternoon
    // set up a random time to go out from dormitory
    time = randomTimeInterval(Time(13, 30, 0), Time(13, 50, 0));

    int afternoon = 0;
    while (it != daily_courses.end()) {
      if (it->start_period > 8)
        break;
      // go to attend the lecture
      daily_schedule.insert(time, AttendClass, it->classroom);

      // also, set up a time range of 5mins
      auto time_end = period_over_time[it->end_period];
      time_end.advance(0, 5, 0);
      time = randomTimeInterval(period_over_time[it->end_period], time_end);

      afternoon++;
      it++;
    }
    if (afternoon == 0) {
      // it does not matter for this meaningless movement
      // since *insert* method has avoided this behavior
      daily_schedule.insert(time, BackToDorm, dorm_name);

      // choose a random time for dinner
      time = randomTimeInterval(Time(17, 30, 0), Time(18, 30, 0));
    }

    // if the lecture ends before 5pm, go back to dormitory at first
    if (time < Time(17, 0, 0)) {
      daily_schedule.insert(time, BackToDorm, dorm_name);

      // similarly, choose a random time for dinner
      time = randomTimeInterval(Time(17, 30, 0), Time(18, 30, 0));
    }
    daily_schedule.insert(time, EatInCanteen,
                          randomChoice("canteen1", "canteen2"));

    // assume that it takes 10 to 30 minutes to have dinner
    time.advance(randomTimeInterval(Time(0, 10, 0), Time(0, 30, 0)));

    // checking whether a student has lectures in the evening
    int evening = 0;
    while (it != daily_courses.end()) {
      if (it->start_period > 11)
        break;
      daily_schedule.insert(time, AttendClass, it->classroom);

      auto time_end = period_over_time[it->end_period];
      time_end.advance(0, 5, 0);
      time = randomTimeInterval(period_over_time[it->end_period], time_end);

      evening++;
      it++;
    }
    if (evening == 0) {
      daily_schedule.insert(time, BackToDorm, dorm_name);

      // a student may need to run in the evening if no evening lectures
      // of course, they may use different transportation instead of on foot
      time = randomTimeInterval(Time(20, 30, 0), Time(21, 30, 0));
      if (rand() % 10 >= 8) {
        // assume that they will need to go to c1 and c15 sequentially
        daily_schedule.insert(time, Run, "c1"); // run to c1
        time.advance(0, 2, 0);
        daily_schedule.insert(time, Run, "c15"); // run to c15
        time.advance(0, 2, 0);
      }
    }
    // run back to dorm
    daily_schedule.insert(time, BackToDorm, dorm_name);
    weekly_schedule->insert(daily_schedule);
  }
  // mark # of week of this weekly schedule to avoid meaningless update
  assert(weekly_schedule->sub_schedules.size() == 7);
  return weekly_schedule;
}
