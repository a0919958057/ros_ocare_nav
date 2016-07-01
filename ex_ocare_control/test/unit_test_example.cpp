#include <gtest/gtest.h>

TEST(DiffStage, stageChangeDetector){
EXPECT_EQ(5, 5);
EXPECT_EQ(5, 10);
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
