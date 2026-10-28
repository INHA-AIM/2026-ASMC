// Offline policy sensitivity only. Reuses the exact current planner core.
#include "local/planner.hpp"
#include <iomanip>
#include <iostream>
using namespace local_planning;
Scene scene() {
  Scene s;
  s.grid.rows = 250;
  s.grid.cols = 160;
  s.grid.probability.assign(40000, 255);
  s.grid.coverage.assign(40000, 255);
  for (int i = 0; i <= 40; ++i) {
    s.corridor.push_back({i * .5, 5, -5});
    s.home.push_back({i * .5, 0});
  }
  s.age = .18;
  return s;
}
int main() {
  std::cout << std::setprecision(12);
  for (const char *name : {"clear", "hood", "spurious_large_obb"})
    for (double kph : {0., 10., 25.}) {
      auto s = scene();
      s.speed = kph / 3.6;
      Config c;
      c.budget_ms = 500.;
      c.recovery = true;
      if (std::string(name) == "hood")
        for (int r = 137; r < 220; ++r)
          for (int col = 70; col < 90; ++col)
            s.grid.coverage[r * 160 + col] = 128;
      if (std::string(name) == "spurious_large_obb")
        s.obstacles.push_back(
            {999, {{5., -4.}, {8., -4.}, {8., 4.}, {5., 4.}}});
      auto result = Plan(s, c);
      std::cout << "{\"case\":\"" << name << "\",\"speed_kph\":" << kph
                << ",\"valid\":" << (result.valid ? "true" : "false")
                << ",\"reason\":\"" << result.reason << "\",\"speed_reason\":\""
                << result.speed_reason
                << "\",\"cap_kph\":" << result.chosen_cap * 3.6
                << ",\"path_m\":"
                << (result.path.empty() ? 0 : result.path.back().s)
                << ",\"recovery\":" << (result.recovery ? "true" : "false")
                << "}\n";
    }
}
