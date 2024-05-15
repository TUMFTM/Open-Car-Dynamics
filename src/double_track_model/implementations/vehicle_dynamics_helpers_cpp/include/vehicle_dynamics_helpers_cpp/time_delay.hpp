// Copyright 2023 Simon Sagmeister
#pragma once
#include <chrono>
#include <queue>
namespace tam::sim::helpers::time_delay
{
template <typename T>
class TimeDelay
{
  struct StampedSignal
  {
    uint64_t stamp;
    T value;
    StampedSignal(uint64_t stamp_, T value_)
    {
      stamp = stamp_;
      value = value_;
    }
  };

public:
  TimeDelay() { TimeDelay(0, T()); }
  TimeDelay(uint64_t delay_microseconds, const T & initial_value)
  {
    set_delay(delay_microseconds);
    reset(initial_value);
  }
  void set(uint64_t current_time_microseconds, const T & value)
  {
    signal_queue_.emplace(current_time_microseconds, value);
  }
  T get(uint64_t current_time_microseconds)
  {
    update_active_signal(current_time_microseconds);
    return active_signal_;
  }
  void set_delay(uint64_t delay_microseconds) { delay_microseconds_ = delay_microseconds; }
  void reset(const T & initial_value)
  {
    while (!signal_queue_.empty()) signal_queue_.pop();
    active_signal_ = initial_value;
  }

private:
  uint64_t delay_microseconds_;
  T active_signal_;
  std::queue<StampedSignal> signal_queue_;
  void update_active_signal(uint64_t current_time_microseconds)
  {
    while (true) {
      if (signal_queue_.empty()) break;
      StampedSignal element = signal_queue_.front();
      if (current_time_microseconds - element.stamp < delay_microseconds_) break;
      active_signal_ = element.value;
      signal_queue_.pop();
    }
  }
};
}  // namespace tam::sim::helpers::time_delay
