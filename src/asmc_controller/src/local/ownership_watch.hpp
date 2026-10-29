#pragma once
// LOCAL only. Never perform ROS master I/O on the 20Hz control callback.
#include <ros/master.h>
#include <ros/this_node.h>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <algorithm>
#include <utility>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace asmc_local {
class OwnershipWatch {
public:
  struct Snapshot {
    bool verified = false, conflict = false;
    std::string reason = "ownership_initializing";
    double age_s = -1, query_ms = 0;
  };
  OwnershipWatch(std::string command, std::string normal, std::string selector,
                 std::string host = ros::master::getHost(), int port = ros::master::getPort())
      : command_(std::move(command)), normal_(std::move(normal)),
        selector_(std::move(selector)), caller_(ros::this_node::getName()),
        host_(std::move(host)), port_(port),
        worker_([this] { run(); }) {}
  ~OwnershipWatch() {
    { std::lock_guard<std::mutex> lock(mutex_); done_ = true; }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
  }
  Snapshot read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto out = value_;
    if (checked_ != Clock::time_point{}) {
      out.age_s = std::chrono::duration<double>(Clock::now() - checked_).count();
      if (out.age_s > 1.25) {
        out.verified = false;
        out.reason = "ownership_stale";
      }
    }
    return out;
  }
private:
  using Clock = std::chrono::steady_clock;
  std::string command_, normal_, selector_, caller_;
  std::string host_;
  int port_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  bool done_ = false;
  Snapshot value_;
  Clock::time_point checked_{};
  std::thread worker_;

  Snapshot query() {
    Snapshot out;
    out.reason = "ownership_master_unavailable";
    const auto start = Clock::now();
    XmlRpc::XmlRpcClient client(host_.c_str(), port_);
    XmlRpc::XmlRpcValue args, result;
    args[0] = caller_;
    bool complete = false;
    if (client.executeNonBlock("getSystemState", args)) {
      while (Clock::now() - start < std::chrono::milliseconds(200)) {
        { std::lock_guard<std::mutex> lock(mutex_); if (done_) break; }
        client._disp.work(0.005);
        if (client.executeCheckDone(result)) { complete = true; break; }
      }
    }
    client.close();
    out.query_ms = std::chrono::duration<double, std::milli>(Clock::now()-start).count();
    if (!complete || client.isFault()) return out;
    try {
      if (result.size() != 3 || int(result[0]) != 1 || result[2].size() != 3)
        return out;
      const auto &publishers = result[2][0];
      for (int i = 0; i < publishers.size(); ++i) {
        const std::string topic = publishers[i][0];
        if (topic != command_ && topic != "/ctrl_cmd/normal" &&
            !(topic == "/ctrl_cmd" && !selector_.empty())) continue;
        for (int j = 0; j < publishers[i][1].size(); ++j) {
          const std::string owner = publishers[i][1][j];
          const bool allowed = topic == "/ctrl_cmd/normal" ?
              (!normal_.empty() && owner == normal_) :
              topic == "/ctrl_cmd" ? owner == selector_ : owner == caller_;
          if (!allowed) {
            out.conflict = true;
            out.reason = topic + " owned by " + owner;
            return out;
          }
        }
      }
      out.verified = true;
      out.reason = "ok";
    } catch (const XmlRpc::XmlRpcException &) { }
    return out;
  }
  void run() {
    auto next = Clock::now();
    while (true) {
      const auto result = query();
      std::unique_lock<std::mutex> lock(mutex_);
      value_ = result;
      checked_ = Clock::now();
      next = std::max(next + std::chrono::seconds(1), checked_);
      if (wake_.wait_until(lock, next, [this] { return done_; })) return;
    }
  }
};
} // namespace asmc_local
