#include "trajectory_reader_rc_control.hpp"

#include <gtest/gtest.h>

TEST(TrajectoryReaderRcControlTest, QuantizeAuxSelectionRejectsInvalidInputs)
{
	EXPECT_EQ(quantizeAuxSelection(NAN, 3), -1);
	EXPECT_EQ(quantizeAuxSelection(0.0f, 0), -1);
}

TEST(TrajectoryReaderRcControlTest, QuantizeAuxSelectionSplitsRangeIntoSlots)
{
	EXPECT_EQ(quantizeAuxSelection(-1.0f, 2), 0);
	EXPECT_EQ(quantizeAuxSelection(0.0f, 2), 1);
	EXPECT_EQ(quantizeAuxSelection(0.99f, 6), 5);
}

TEST(TrajectoryReaderRcControlTest, RcButtonPressedUsesOneBasedButtonIndex)
{
	EXPECT_TRUE(rcButtonPressed(0b0001, 1));
	EXPECT_FALSE(rcButtonPressed(0b0001, 2));
	EXPECT_TRUE(rcButtonPressed(0x8000, 16));
	EXPECT_FALSE(rcButtonPressed(0x8000, 17));
}

TEST(TrajectoryReaderRcControlTest, TrajectorySelectionUsesConfiguredIdRange)
{
	EXPECT_EQ(trajectorySelectionSlotCount(100, 104), 5);
	EXPECT_EQ(trajectoryIdFromSelectionSlot(0, 100, 104), 100);
	EXPECT_EQ(trajectoryIdFromSelectionSlot(4, 100, 104), 104);
	EXPECT_EQ(trajectoryIdFromSelectionSlot(99, 100, 104), 104);
}
