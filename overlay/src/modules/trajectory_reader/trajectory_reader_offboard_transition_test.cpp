#include <gtest/gtest.h>

#include "trajectory_reader_offboard_transition.hpp"

TEST(TrajectoryReaderOffboardTransitionTest, RelativeReferenceIsReanchoredButPreserved)
{
	const auto reset = position_mode_offboard_entry_reset(false);

	EXPECT_TRUE(reset.need_pos_offset);
	EXPECT_FALSE(reset.pos_offset_valid);
	EXPECT_FALSE(reset.preserve_absolute_reference);
	EXPECT_TRUE(reset.preserve_target);
}

TEST(TrajectoryReaderOffboardTransitionTest, AbsoluteReferenceIsPreserved)
{
	const auto reset = position_mode_offboard_entry_reset(true);

	EXPECT_FALSE(reset.need_pos_offset);
	EXPECT_FALSE(reset.pos_offset_valid);
	EXPECT_TRUE(reset.preserve_absolute_reference);
	EXPECT_TRUE(reset.preserve_target);
}
