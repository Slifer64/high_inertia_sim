#ifndef HIGH_INERTIA_SIM_UNITS_H
#define HIGH_INERTIA_SIM_UNITS_H

#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <functional>
#include <condition_variable>


inline arma::vec getPositionFromTf(const arma::mat &Tf)
{
  return Tf.submat(0,3,2,3);
}

inline arma::mat getRotmFromTf(const arma::mat &Tf)
{
  return Tf.submat(0,0,2,2);
}

template<typename T>
class MtxVar
{
public:
  MtxVar() { }
  MtxVar& operator=(const T &val) { set(val); return *this; }
  T operator()() const { return get(); }
  T get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  T read() const { return var; }
  void set(const T &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  T var;
};

// specialization for bool
template<>
class MtxVar<bool>
{
public:
  MtxVar() { var = false; }
  MtxVar& operator=(const bool &val) { set(val); return *this; }
  bool operator()() const { return get(); }
  bool get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  bool read() const { return var; }
  void set(const bool &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  bool var;
};

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:
  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool wait_until(double time_ms)
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);

    int wait_time = time_ms * 1e6;

    while(!count_) // Handle spurious wake-ups.
    {
      if (condition_.wait_for(lock,std::chrono::nanoseconds(wait_time))==std::cv_status::timeout)
        return false;
    }
    // --count_;
    count_ = false;

    return true;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
};

class Timer
{
public:
  Timer()
  {
    clock_gettime(CLOCK_REALTIME, &beg_);
  }

  long int elapsedNanoSec()
  {
    clock_gettime(CLOCK_REALTIME, &end_);
    return (end_.tv_sec - beg_.tv_sec)*1000000000 + (end_.tv_nsec - beg_.tv_nsec);
  }

  double elapsedMicroSec()
  {
    return elapsedNanoSec()/1000.0;
  }

  double elapsedMilliSec()
  {
    return elapsedNanoSec()/1000000.0;
  }

  double elapsedSec()
  {
    return elapsedNanoSec()/1000000000.0;
  }

  void start() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
  timespec beg_, end_;
};


#endif // HIGH_INERTIA_SIM_UNITS_H
