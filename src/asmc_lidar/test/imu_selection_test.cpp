#include <asmc_lidar/core/imu_selection.hpp>
#include <cassert>
#include <deque>
#include <iostream>
using asmc_lidar::ImuSample;
using asmc_lidar::ImuSelectionStatus;
using asmc_lidar::SelectImu;
int main(){
  std::deque<ImuSample> samples;
  assert(SelectImu(samples,200).status==ImuSelectionStatus::Missing);
  samples.push_back({250,1,1,true});
  assert(SelectImu(samples,200).status==ImuSelectionStatus::FutureOnly);
  samples.push_back({180,.2,.3,true});
  samples.push_back({160,.4,.5,true}); // arrival order is not stamp order.
  auto chosen=SelectImu(samples,200);
  assert(chosen.status==ImuSelectionStatus::Valid);
  assert(chosen.sample.stamp_ns==180 && chosen.sample.roll_rad==.2);
  samples.push_back({190,0,0,false});
  chosen=SelectImu(samples,200);
  assert(chosen.status==ImuSelectionStatus::Valid);
  assert(chosen.sample.stamp_ns==180 && chosen.newer_invalid_seen);
  samples.push_back({190,.6,.7,true});
  chosen=SelectImu(samples,200);
  assert(chosen.sample.roll_rad==.6 && !chosen.newer_invalid_seen);
  std::deque<ImuSample> invalid_only{{190,0,0,false}};
  assert(SelectImu(invalid_only,200).status==ImuSelectionStatus::Invalid);
  std::deque<ImuSample> unstamped{{0,0,0,false}};
  assert(SelectImu(unstamped,200).status==ImuSelectionStatus::Missing);
  assert(SelectImu(samples,100000251,100000000).status==ImuSelectionStatus::Stale);
  assert(SelectImu(samples,100000251,100000000).sample.stamp_ns==250);
  std::deque<ImuSample> boundary{{100,.1,.2,true}};
  assert(SelectImu(boundary,100000100).status==ImuSelectionStatus::Valid);
  assert(SelectImu(boundary,100000101).status==ImuSelectionStatus::Stale);
  std::cout<<"imu_selection_test passed\n";
}
