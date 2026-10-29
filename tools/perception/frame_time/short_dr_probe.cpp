// Offline probe only: the SAME local MotionHistory and shared yaw convention.
// No GPS/map publisher, controller or simulator access.
#include "asmc_geometry/geodesy.hpp"
#include "local/motion.hpp"
#include <iomanip>
#include <iostream>
int main() {
  local_planning::MotionHistory history;
  std::cout << std::setprecision(17);
  double t, v, x, y, z, w, gx, gy, gz;
  while (std::cin >> t >> v >> x >> y >> z >> w >> gx >> gy >> gz) {
    const double roll =
        std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2 * (w * y - z * x), -1., 1.));
    const double rate =
        (std::sin(roll) * gy + std::cos(roll) * gz) / std::cos(pitch);
    if (history.add(t, v, rate, roll, pitch)) {
      auto m = history.at(t);
      if (m)
        std::cout << t << ' ' << m->x << ' ' << m->y << ' ' << m->yaw << ' '
                  << history.epoch << '\n';
    }
  }
}
