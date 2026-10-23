#include <asmc_lidar/core/source_time.hpp>
#include <asmc_lidar/core/processing_epoch.hpp>
#include <gtest/gtest.h>
#include <ros/time.h>
#include <mutex>
#include <thread>
#include <condition_variable>

TEST(SourceTime, MatchesRosDurationIncludingNanosecondBorrow) {
  const std::int64_t times[] = {0, 1, 999999999, 1000000000, 1790654785683737993LL,
                               1790654785700000001LL, 2000000000000000000LL};
  for (auto current : times) for (auto previous : times) {
    const auto expected = (ros::Time().fromNSec(current) - ros::Time().fromNSec(previous)).toSec();
    EXPECT_DOUBLE_EQ(asmc::lidar::pipeline::SourceDeltaSeconds(current, previous), expected);
  }
}
TEST(SourceTime, RejectsUnsupportedRosTimeAndDurationRange) {
  using asmc::lidar::pipeline::SourceDeltaSeconds;
  EXPECT_THROW(SourceDeltaSeconds(-1, 0), std::out_of_range);
  EXPECT_THROW(SourceDeltaSeconds(4294967296000000000LL, 0), std::out_of_range);
  EXPECT_THROW(SourceDeltaSeconds(3000000000000000000LL, 0), std::out_of_range);
}
TEST(ProcessingEpoch, ResetRetiresAnInFlightResultAndRestartChangesSession) {
  asmc::lidar::runtime::ProcessingEpoch epoch, restarted;
  EXPECT_NE(epoch.session(), restarted.session());
  const auto session = epoch.session();
  std::mutex mutex; std::condition_variable cv; bool started=false, reset=false, published=false;
  std::thread worker([&] {
    std::unique_lock<std::mutex> lock(mutex); const auto ticket = epoch.generation();
    started=true; cv.notify_all(); cv.wait(lock,[&]{return reset;});
    published=epoch.IsCurrent(ticket);
  });
  { std::unique_lock<std::mutex> lock(mutex); cv.wait(lock,[&]{return started;});
    epoch.Reset(); reset=true; cv.notify_all(); }
  worker.join(); EXPECT_FALSE(published); EXPECT_EQ(session,epoch.session());
  EXPECT_EQ(epoch.generation(),2u); EXPECT_TRUE(epoch.IsCurrent(2));
}

int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
