#include "lane_boundary_adapter/obstacle_memory.hpp"
#include <gtest/gtest.h>
using namespace lane_boundary_adapter::obstacle_memory;
namespace {
Obstacle box(int id,double left,double right,double stamp=1.){
    Obstacle o{10,2,left,right,0,1};o.track_id=id;
    o.source="producer/1";o.last_seen=stamp;o.label="person";return o;
}
TEST(ObstacleMemory,SameIdReplacesPositionAndShrinksShape){
    std::vector<Obstacle> memory{box(7,1,4)};
    updateLidar(memory,{box(7,2,3,1.1)},"producer/1",3);
    ASSERT_EQ(memory.size(),1u);EXPECT_DOUBLE_EQ(memory[0].line_left,2);
    EXPECT_DOUBLE_EQ(memory[0].line_right,3);EXPECT_EQ(memory[0].hits,2);
}
TEST(ObstacleMemory,SimultaneousDifferentObjectsRemainDistinct){
    std::vector<Obstacle> memory;
    updateLidar(memory,{box(7,2,3),box(8,4,5)},"producer/1",3);
    EXPECT_EQ(memory.size(),2u);
}
TEST(ObstacleMemory,ChangedIdDoesNotLeaveRetiredIdBehind){
    std::vector<Obstacle> memory{box(7,2,3)};
    updateLidar(memory,{box(8,3,4,1.1)},"producer/1",3);
    ASSERT_EQ(memory.size(),1u);EXPECT_EQ(memory[0].track_id,8);EXPECT_EQ(memory[0].hits,1);
}
TEST(ObstacleMemory,ProducerEmptySnapshotReleasesRetiredTracks){
    std::vector<Obstacle> memory{box(7,2,3)};
    updateLidar(memory,{},"producer/1",3);EXPECT_TRUE(memory.empty());
}
TEST(ObstacleMemory,ProducerPredictionPreservesConfirmedTrackWithoutAddingHits){
    auto old=box(7,2,3);old.hits=3;
    std::vector<Obstacle> memory{old};auto predicted=box(7,3,4,1.1);predicted.predicted=true;
    updateLidar(memory,{predicted},"producer/1",3);
    ASSERT_EQ(memory.size(),1u);EXPECT_TRUE(memory[0].predicted);
    EXPECT_EQ(memory[0].hits,3);EXPECT_DOUBLE_EQ(memory[0].line_left,3);
}
TEST(ObstacleMemory,NewPredictionCannotConfirmItself){
    std::vector<Obstacle> memory;auto predicted=box(7,2,3);predicted.predicted=true;
    updateLidar(memory,{predicted},"producer/1",3);EXPECT_EQ(memory[0].hits,0);
    updateLidar(memory,{predicted},"producer/1",3);EXPECT_EQ(memory[0].hits,0);
}
TEST(ObstacleMemory,ProducerRestartReplacesOldSessionAndResetsConfirmation){
    auto old=box(7,2,3);old.hits=3;std::vector<Obstacle> memory{old};
    updateLidar(memory,{box(7,4,5)},"producer/2",3);
    ASSERT_EQ(memory.size(),1u);EXPECT_EQ(memory[0].source,"producer/2");EXPECT_EQ(memory[0].hits,1);
}
TEST(ObstacleMemory,RepeatedIdChangesDoNotAccumulateHistory){
    std::vector<Obstacle> memory;
    for(int id=0;id<100;++id){
        updateLidar(memory,{box(id,2,3)},"producer/1",3);
        ASSERT_EQ(memory.size(),1u);EXPECT_EQ(memory[0].track_id,id);
    }
}
}
int main(int argc,char** argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
