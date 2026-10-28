#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace local_planning {
// No ROS callback writes to disk. Lost records are explicit test failures.
class Journal {
public:
  std::atomic<unsigned> dropped{0};
  explicit Journal(const std::string &path) {
    if (path.empty())
      return;
    std::ifstream existing(path);
    if (existing.good())
      throw std::runtime_error("journal already exists");
    stream.open(path);
    if (!stream)
      throw std::runtime_error("cannot open registered journal");
    worker = std::thread([this] { run(); });
  }
  void push(std::string row) {
    if (!worker.joinable())
      return;
    std::lock_guard<std::mutex> g(mutex);
    if (queue.size() >= 256) {
      ++dropped;
      return;
    }
    queue.push_back(std::move(row));
    ready.notify_one();
  }
  ~Journal() {
    {
      std::lock_guard<std::mutex> g(mutex);
      done = true;
    }
    ready.notify_one();
    if (worker.joinable())
      worker.join();
  }

private:
  void run() {
    while (true) {
      std::string row;
      {
        std::unique_lock<std::mutex> g(mutex);
        ready.wait(g, [this] { return done || !queue.empty(); });
        if (queue.empty() && done)
          break;
        row = std::move(queue.front());
        queue.pop_front();
      }
      stream << row << '\n';
      stream.flush();
      if (!stream)
        ++dropped;
    }
  }
  std::ofstream stream;
  std::thread worker;
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<std::string> queue;
  bool done = false;
};
} // namespace local_planning
