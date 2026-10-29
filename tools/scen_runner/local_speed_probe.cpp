// Offline native Plan probe. Fixed free raster; not simulator performance.
#include "local/planner.hpp"
#include <iostream>
using namespace local_planning;
int main() {
  Config c;
  c.budget_ms = 500;
  c.speed_cap = 15. / 3.6;
  Scene s;
  s.grid.rows = 250;
  s.grid.cols = 160;
  s.grid.probability.assign(40000, 255);
  s.grid.coverage.assign(40000, 255);
  for (int i = 0; i <= 40; i++) {
    s.corridor.push_back({i * .5, 5, -5});
    s.home.push_back({i * .5, 0});
  }
  for (double v : {0., 2.8, 4.0, 4.1666666667, 4.3, 4.6})
    for (double a : {-1., 0., 1.}) {
      s.speed = v;
      s.acceleration = a;
      auto r = Plan(s, c);
      std::cout << v << ',' << a << ',' << r.valid << ',' << r.path.back().s
                << ',' << r.chosen_cap;
      for (int i : {2, 10, 20, 40, 50}) {
        if (r.trajectory.empty())
          break;
        auto q = r.trajectory[i];
        std::cout << ',' << q[4] << ',' << q[5];
      }
      std::cout << '\n';
    }
}
