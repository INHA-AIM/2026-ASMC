#include "Frenet/Generator/SamplingUtils.hpp"

std::vector<double> SampleRange(const SamplingRange& r) {
    std::vector<double> vals;
    if (r.step <= 1e-9) {
        vals.push_back(r.min);
        return vals;
    }
    for (double v = r.min; v <= r.max + 1e-9; v += r.step) {
        vals.push_back(v);
    }
    return vals;
}

std::vector<double> SampleTimes(double T, double dt) {
    std::vector<double> times;
    for (double t = 0.0; t < T - 1e-9; t += dt) {
        times.push_back(t);
    }
    times.push_back(T);
    return times;
}
