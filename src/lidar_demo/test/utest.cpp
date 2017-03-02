#include "gtest/gtest.h"

#include "lidar_demo/lidar_demo.h"


TEST(TestLidarDemo, emptyTest)
{
	EXPECT_EQ(1, 1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
