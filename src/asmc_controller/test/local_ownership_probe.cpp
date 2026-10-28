// Test-only probe: a slow fake master must never hold the control-side read.
#include "local/ownership_watch.hpp"
#include <ros/init.h>
#include <iostream>
#include <iomanip>
int main(int argc, char **argv) {
  ros::init(argc, argv, "local_ownership_probe", ros::init_options::AnonymousName);
  if (argc != 2) return 2;
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  {
    asmc_local::OwnershipWatch watch("/ctrl_cmd/jamming", "", "", "127.0.0.1", std::stoi(argv[1]));
    while (Clock::now()-start < std::chrono::seconds(4)) {
      const auto before = Clock::now();
      const auto value = watch.read();
      const double cost = std::chrono::duration<double,std::milli>(Clock::now()-before).count();
      std::cout << std::setprecision(12) << "{\"t\":"
        << std::chrono::duration<double>(Clock::now()-start).count()
        << ",\"read_ms\":" << cost << ",\"verified\":" << value.verified
        << ",\"conflict\":" << value.conflict << ",\"query_ms\":" << value.query_ms << "}" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  return std::chrono::duration<double>(Clock::now()-start).count() > 4.4 ? 1 : 0;
}
