#pragma once

#include <cmath>
#include <cstdint>

#include <matrix/matrix/math.hpp>

enum class RcWorkflowSelection : uint8_t {
	HOLD_POSITION = 0,
	SINGLE_IDENT = 1,
	SINGLE_TRAJECTORY = 2,
	IDENTIFICATION_ONLY = 3,
	TRAJECTORY_ONLY = 4,
	FULL_STACK = 5
};

inline int quantizeAuxSelection(float aux_value, int slots)
{
	if (!std::isfinite(static_cast<double>(aux_value)) || slots <= 0) {
		return -1;
	}

	const float normalized = math::constrain((aux_value + 1.0f) * 0.5f, 0.0f, 0.999999f);
	return math::constrain(static_cast<int>(floorf(normalized * static_cast<float>(slots))), 0, slots - 1);
}

inline bool rcButtonPressed(uint16_t buttons, int button_index)
{
	if (button_index < 1 || button_index > 16) {
		return false;
	}

	return (buttons & (1u << (button_index - 1))) != 0;
}

inline int32_t trajectorySelectionSlotCount(int32_t min_id, int32_t max_id)
{
	const int32_t clamped_min = math::max<int32_t>(0, min_id);
	const int32_t clamped_max = math::max<int32_t>(clamped_min, max_id);
	return math::max<int32_t>(1, clamped_max - clamped_min + 1);
}

inline uint8_t trajectoryIdFromSelectionSlot(int selection_slot, int32_t min_id, int32_t max_id)
{
	const int32_t clamped_min = math::max<int32_t>(0, min_id);
	const int32_t clamped_max = math::max<int32_t>(clamped_min, max_id);
	const int32_t slot_count = trajectorySelectionSlotCount(clamped_min, clamped_max);
	const int32_t clamped_slot = math::constrain<int32_t>(selection_slot, 0, slot_count - 1);
	return static_cast<uint8_t>(math::constrain<int32_t>(clamped_min + clamped_slot, clamped_min, clamped_max));
}
