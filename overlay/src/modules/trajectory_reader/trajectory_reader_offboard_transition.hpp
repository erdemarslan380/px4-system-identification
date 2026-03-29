#pragma once

struct PositionModeOffboardEntryReset {
	bool need_pos_offset{false};
	bool pos_offset_valid{false};
	bool preserve_absolute_reference{false};
	bool preserve_target{true};
};

inline constexpr PositionModeOffboardEntryReset position_mode_offboard_entry_reset(bool pos_ref_absolute)
{
	return PositionModeOffboardEntryReset{
		.need_pos_offset = !pos_ref_absolute,
		.pos_offset_valid = false,
		.preserve_absolute_reference = pos_ref_absolute,
		.preserve_target = true,
	};
}
