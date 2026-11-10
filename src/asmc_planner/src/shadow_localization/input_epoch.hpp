#pragma once
#include <cmath>
#include <cstdint>
#include <set>
#include <string>

namespace shadow_localization {
// Producer identity is independent of wall-clock freshness. Retired producers
// cannot switch the consumer back after a restart. Invalid geometry is handled
// by the consumer even if the identity and sequence themselves are acceptable.
class InputEpoch {
 public:
  enum class Result { accepted, changed, duplicate, rejected };
  std::string session, config, calibration, reason = "uninitialized";
  uint64_t generation = 0, sequence = 0;
  double stamp = 0;
  unsigned rejected = 0;
  Result accept(const std::string& s, uint64_t g, uint64_t n,
                const std::string& c, const std::string& k, double t) {
    auto reject = [&](const char* why) { reason=why; ++rejected; return Result::rejected; };
    if (s.empty() || c.empty() || k.empty() || !std::isfinite(t) || t<=0)
      return reject("identity_incomplete");
    if (retired_.count(s)) return reject("retired_producer");
    if (s==session) {
      if (g<generation) return reject("retired_generation");
      if (g==generation && n==sequence && t==stamp && c==config && k==calibration)
        return Result::duplicate;
      if (g==generation && (n<=sequence || t<=stamp)) return reject("scan_time_or_sequence_regression");
    }
    const bool changed=!session.empty() && (s!=session || g!=generation || c!=config || k!=calibration);
    if (!session.empty() && s!=session) retired_.insert(session);
    session=s; generation=g; sequence=n; config=c; calibration=k; stamp=t;
    reason=changed?"epoch_changed":"accepted";
    return changed?Result::changed:Result::accepted;
  }
 private:
  std::set<std::string> retired_;
};
}  // namespace shadow_localization
