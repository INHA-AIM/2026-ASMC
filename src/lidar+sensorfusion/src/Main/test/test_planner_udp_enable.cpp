// ~planner_udp_enabled seam (PlannerInterface/PlannerObjectSender.hpp). No test opens an
// AF_INET socket or sends a datagram: disabled factory paths must create no socket at all, and the
// descriptor counter is proven sensitive with a file and an AF_UNIX socketpair only.
#include <gtest/gtest.h>

#include <PlannerInterface/PlannerObjectSender.hpp>
#include <ros/package.h>

#include <dirent.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <string>

namespace {
struct FdCount { int total = 0, sockets = 0; };
FdCount OpenDescriptors() {
  FdCount count;
  DIR* dir = opendir("/proc/self/fd");
  if (!dir) return {-1, -1};
  const int own = dirfd(dir);
  while (const dirent* entry = readdir(dir)) {
    if (entry->d_name[0] == '.') continue;
    const int fd = std::atoi(entry->d_name);
    if (fd == own) continue;
    ++count.total;
    char target[256] = {};
    const std::string link = std::string("/proc/self/fd/") + entry->d_name;
    const ssize_t size = readlink(link.c_str(), target, sizeof(target) - 1);
    if (size > 0 && std::strncmp(target, "socket:", 7) == 0) ++count.sockets;
  }
  closedir(dir);
  return count;
}
const char* kEnabledYaml = "{Enabled: true, IP: 127.0.0.1, Port: 7510, MaxObjects: 20}";
}  // namespace

TEST(PlannerUdpEnable, DescriptorCounterDetectsNewDescriptorsAndSockets) {
  const FdCount before = OpenDescriptors();
  ASSERT_GE(before.total, 0);
  const int file = open("/dev/null", O_RDONLY);
  ASSERT_GE(file, 0);
  int pair[2] = {-1, -1};
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, pair), 0);   // local IPC only, never AF_INET
  const FdCount during = OpenDescriptors();
  EXPECT_EQ(during.total, before.total + 3);
  EXPECT_EQ(during.sockets, before.sockets + 2);
  close(file); close(pair[0]); close(pair[1]);
  const FdCount after = OpenDescriptors();
  EXPECT_EQ(after.total, before.total);
  EXPECT_EQ(after.sockets, before.sockets);
}

TEST(PlannerUdpEnable, DefaultReproducesProduction) {
  EXPECT_TRUE(kPlannerUdpEnabledDefault);
  // With the default the YAML decides, exactly as before the parameter existed; production
  // setting.yaml enables the sender.
  const std::string path = ros::package::getPath("Main") + "/setting.yaml";
  const YAML::Node planner = YAML::LoadFile(path)["PlannerInterface"];
  ASSERT_TRUE(planner) << path;
  EXPECT_TRUE(planner["Enabled"].as<bool>(false));
  EXPECT_EQ(planner["Port"].as<int>(), 7510);
}

TEST(PlannerUdpEnable, DisabledCreatesNoSocket) {
  const FdCount before = OpenDescriptors();
  auto sender = MakePlannerObjectSender(false, YAML::Load(kEnabledYaml));
  EXPECT_TRUE(sender == nullptr);
  const FdCount after = OpenDescriptors();
  EXPECT_EQ(after.total, before.total);
  EXPECT_EQ(after.sockets, before.sockets);
  // Production setting.yaml (Enabled: true) with planner_udp_enabled=false: still no sender.
  const YAML::Node production =
      YAML::LoadFile(ros::package::getPath("Main") + "/setting.yaml")["PlannerInterface"];
  EXPECT_TRUE(MakePlannerObjectSender(false, production) == nullptr);
  EXPECT_EQ(OpenDescriptors().sockets, before.sockets);
}

TEST(PlannerUdpEnable, YamlDisabledOrMissingCreatesNoSocket) {
  const FdCount before = OpenDescriptors();
  EXPECT_TRUE(MakePlannerObjectSender(true, YAML::Load("{Enabled: false, IP: 127.0.0.1, Port: 7510}")) == nullptr);
  EXPECT_TRUE(MakePlannerObjectSender(true, YAML::Load("{IP: 127.0.0.1, Port: 7510}")) == nullptr);
  EXPECT_TRUE(MakePlannerObjectSender(true, YAML::Node()) == nullptr);
  EXPECT_TRUE(MakePlannerObjectSender(true, YAML::Load("{}")["PlannerInterface"]) == nullptr);
  const FdCount after = OpenDescriptors();
  EXPECT_EQ(after.total, before.total);
  EXPECT_EQ(after.sockets, before.sockets);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
