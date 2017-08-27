/*                                                                         80->|
 * timer.cpp
 *
 * Modifications: James William Dunn
 *          Date: August 26, 2017
 */

#include "timer.h"

Timer1::Timer1() {
  start();
  beginning_ = start_;
}
Timer1::~Timer1() {}

void Timer1::start() {
  start_ = high_resolution_clock::now();
}

void Timer1::restart() {
	start();
	beginning_ = high_resolution_clock::now();
}

Timer1::ms Timer1::duration() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  Timer1::ms durat = std::chrono::duration_cast<ms>(now - start_);
  start_ = now;
  return durat;
}

Timer1::ms Timer1::since() {
	high_resolution_clock::time_point now = high_resolution_clock::now();
	Timer1::ms durat = std::chrono::duration_cast<ms>(now - start_);
	return durat;
}

long Timer1::sinceBeginning() {
	high_resolution_clock::time_point now = high_resolution_clock::now();
	Timer1::ms t = std::chrono::duration_cast<ms>(now - beginning_);
	return t.count();
}