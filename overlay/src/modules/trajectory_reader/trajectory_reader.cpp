/**
 * @file trajectory_reader.cpp
 * 
 * Implements the TrajectoryReader scheduled work item.
 * Handles file I/O, trajectory buffering, and periodic publication.
 */


#include "trajectory_reader.hpp"
#include "trajectory_reader_offboard_transition.hpp"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

namespace
{
TrajectoryReader *trajectoryReaderInstance()
{
	return ModuleBase::get_instance<TrajectoryReader>(TrajectoryReader::desc);
}

float readAuxChannel(const manual_control_setpoint_s &manual_control, int channel)
{
	switch (channel) {
	case 1: return manual_control.aux1;
	case 2: return manual_control.aux2;
	case 3: return manual_control.aux3;
	case 4: return manual_control.aux4;
	case 5: return manual_control.aux5;
	case 6: return manual_control.aux6;
	default: return NAN;
	}
}

int quantizeAuxSelection(float aux_value, int slots)
{
	if (!PX4_ISFINITE(aux_value) || slots <= 0) {
		return -1;
	}

	const float normalized = math::constrain((aux_value + 1.0f) * 0.5f, 0.0f, 0.999999f);
	return math::constrain(static_cast<int>(floorf(normalized * static_cast<float>(slots))), 0, slots - 1);
}

float squaref(float value)
{
	return value * value;
}

struct CampaignDefinitionItem {
	Mode mode;
	IdentificationProfile ident_profile;
	uint8_t traj_id;
	const char *name;
};

static constexpr CampaignDefinitionItem kIdentificationOnlyCampaign[] = {
	{Mode::IDENTIFICATION, IdentificationProfile::HOVER_THRUST, 0, "hover_thrust"},
	{Mode::IDENTIFICATION, IdentificationProfile::MASS_VERTICAL, 0, "mass_vertical"},
	{Mode::IDENTIFICATION, IdentificationProfile::ROLL_SWEEP, 0, "roll_sweep"},
	{Mode::IDENTIFICATION, IdentificationProfile::PITCH_SWEEP, 0, "pitch_sweep"},
	{Mode::IDENTIFICATION, IdentificationProfile::YAW_SWEEP, 0, "yaw_sweep"},
	{Mode::IDENTIFICATION, IdentificationProfile::DRAG_X, 0, "drag_x"},
	{Mode::IDENTIFICATION, IdentificationProfile::DRAG_Y, 0, "drag_y"},
	{Mode::IDENTIFICATION, IdentificationProfile::DRAG_Z, 0, "drag_z"},
	{Mode::IDENTIFICATION, IdentificationProfile::MOTOR_STEP, 0, "motor_step"},
};

static constexpr CampaignDefinitionItem kFullStackCampaign[] = {
	{Mode::IDENTIFICATION, IdentificationProfile::HOVER_THRUST, 0, "hover_thrust"},
	{Mode::IDENTIFICATION, IdentificationProfile::MASS_VERTICAL, 0, "mass_vertical"},
	{Mode::IDENTIFICATION, IdentificationProfile::ROLL_SWEEP, 0, "roll_sweep"},
	{Mode::IDENTIFICATION, IdentificationProfile::PITCH_SWEEP, 0, "pitch_sweep"},
	{Mode::IDENTIFICATION, IdentificationProfile::YAW_SWEEP, 0, "yaw_sweep"},
	{Mode::IDENTIFICATION, IdentificationProfile::DRAG_X, 0, "drag_x"},
	{Mode::IDENTIFICATION, IdentificationProfile::DRAG_Y, 0, "drag_y"},
	{Mode::IDENTIFICATION, IdentificationProfile::DRAG_Z, 0, "drag_z"},
	{Mode::IDENTIFICATION, IdentificationProfile::MOTOR_STEP, 0, "motor_step"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 100, "hairpin"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 101, "lemniscate"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 102, "circle"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 103, "time_optimal_30s"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 104, "minimum_snap_50s"},
};

static constexpr CampaignDefinitionItem kTrajectoryOnlyCampaign[] = {
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 100, "hairpin"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 101, "lemniscate"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 102, "circle"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 103, "time_optimal_30s"},
	{Mode::TRAJECTORY, IdentificationProfile::HOVER_THRUST, 104, "minimum_snap_50s"},
};

const CampaignDefinitionItem *campaignDefinition(CampaignType type, size_t &count)
{
	switch (type) {
	case CampaignType::IDENTIFICATION_ONLY:
		count = sizeof(kIdentificationOnlyCampaign) / sizeof(kIdentificationOnlyCampaign[0]);
		return kIdentificationOnlyCampaign;

	case CampaignType::FULL_STACK:
		count = sizeof(kFullStackCampaign) / sizeof(kFullStackCampaign[0]);
		return kFullStackCampaign;

	case CampaignType::TRAJECTORY_ONLY:
		count = sizeof(kTrajectoryOnlyCampaign) / sizeof(kTrajectoryOnlyCampaign[0]);
		return kTrajectoryOnlyCampaign;

	case CampaignType::NONE:
	default:
		count = 0;
		return nullptr;
	}
}
}

ModuleBase::Descriptor TrajectoryReader::desc{task_spawn, custom_command, print_usage};

TrajectoryReader::TrajectoryReader() :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    ModuleParams(nullptr)
{
    PX4_INFO("TrajectoryReader constructed");

    parametersUpdate();
	updateControllerTypeCache();
	_need_pos_offset = true;
}

TrajectoryReader::~TrajectoryReader() {
    PX4_INFO("TrajectoryReader destructed");

    ScheduleClear();
	stopIdentificationLog();
	stopTrajectoryTrackingLog();
	closeTrajectoryFile();
}

bool TrajectoryReader::init() {
	setCampaignStatusParam(0);
    ScheduleOnInterval(20_ms);
    return true;
}

void TrajectoryReader::parametersUpdate() {
    ModuleParams::updateParams();

    setParamMpcAccHor(_param_mpc_acc_hor.get());
	setParamMpcAccDownMax(_param_mpc_acc_down_max.get());
	setParamMpcAccUpMax(_param_mpc_acc_up_max.get());
	setParamMpcJerkAuto(_param_mpc_jerk_auto.get());
	setParamMpcXyCruise(_param_mpc_xy_cruise.get());
	setParamMpcXyErrMax(_param_mpc_xy_err_max.get());
	setParamMpcXyVelMax(_param_mpc_xy_vel_max.get());
	setParamMpcYawrautoMax(_param_mpc_yawrauto_max.get());
	setParamMpcYawrautoAcc(_param_mpc_yawrauto_acc.get());
	setParamMpcZVAutoDn(_param_mpc_z_v_auto_dn.get());
	setParamMpcZVAutoUp(_param_mpc_z_v_auto_up.get());
	_rc_select_enabled = _param_trj_rc_sel_en.get();
	_rc_selector_channel = _param_trj_rc_sel_ch.get();
	_rc_selector_max_traj_id = math::max<int32_t>(0, _param_trj_rc_max_id.get());

	const int32_t ident_profile = math::constrain<int32_t>(_param_trj_ident_prof.get(), 0, 8);
	if (ident_profile != _param_ident_profile_cached) {
		_param_ident_profile_cached = ident_profile;
		_ident_profile = static_cast<IdentificationProfile>(ident_profile);
		stopTrajectoryTrackingLog();
		stopIdentificationLog();
		resetIdentificationState();
		_start_new_tracking_log = (_mode == Mode::IDENTIFICATION);
	}

	const int32_t campaign_type = math::constrain<int32_t>(_param_trj_campaign.get(), 0, 3);
	if (campaign_type != _param_campaign_cached) {
		_param_campaign_cached = campaign_type;
		_campaign_type = static_cast<CampaignType>(campaign_type);

		if (_campaign_active && _campaign_type == CampaignType::NONE) {
			stopCampaign(true);
		}
	}

	const matrix::Vector3f anchor{
		_param_trj_anchor_x.get(),
		_param_trj_anchor_y.get(),
		_param_trj_anchor_z.get()
	};

	if (!_param_anchor_cached_valid || (anchor - _param_anchor_cached).norm_squared() > 1e-6f) {
		_param_anchor_cached = anchor;
		_param_anchor_cached_valid = true;
		setTrajectoryAnchor(anchor);
	}

	const int32_t traj_id = math::constrain<int32_t>(_param_trj_active_id.get(), 0, 255);
	if (traj_id != _param_traj_id_cached) {
		_param_traj_id_cached = traj_id;
		setTrajectoryId(static_cast<uint8_t>(traj_id));
	}

	const int32_t mode_cmd = math::constrain<int32_t>(_param_trj_mode_cmd.get(), 0, 2);
	if (mode_cmd != _param_mode_cmd_cached) {
		_param_mode_cmd_cached = mode_cmd;
		setTrajectoryReaderMode(static_cast<Mode>(mode_cmd));
	}

	const int32_t campaign_cmd = math::constrain<int32_t>(_param_trj_campaign_cmd.get(), 0, 2);
	if (campaign_cmd != _param_campaign_cmd_cached) {
		_param_campaign_cmd_cached = campaign_cmd;

		if (campaign_cmd == 1) {
			requestCampaignStart();
		} else if (campaign_cmd == 2) {
			stopCampaign(true);
		}

		if (campaign_cmd != 0) {
			const int32_t idle = 0;
			param_t campaign_cmd_param = param_find("TRJ_CAMPAIGN_CMD");
			if (campaign_cmd_param != PARAM_INVALID) {
				param_set(campaign_cmd_param, &idle);
			}
			_param_campaign_cmd_cached = 0;
		}
	}
}

void TrajectoryReader::updateControllerTypeCache()
{
	static param_t param_cst_pos_ctrl_typ = PARAM_INVALID;
	static bool param_lookup_done = false;

	if (!param_lookup_done) {
		param_cst_pos_ctrl_typ = param_find("CST_POS_CTRL_TYP");
		param_lookup_done = true;
	}

	if (param_cst_pos_ctrl_typ == PARAM_INVALID) {
		return;
	}

	int32_t controller_type = _controller_type_cached;

	if (param_get(param_cst_pos_ctrl_typ, &controller_type) == PX4_OK) {
		_controller_type_cached = controller_type;
	}
}

const char *TrajectoryReader::controllerTypeToString(int32_t controller_type) const
{
	switch (controller_type) {
	case 4:
		return "px4_default";

	case 6:
		return "sysid";

	case 0:
	default:
		return "none";
	}
}

const char *TrajectoryReader::identProfileToString(IdentificationProfile profile) const
{
	switch (profile) {
	case IdentificationProfile::HOVER_THRUST:
		return "hover_thrust";
	case IdentificationProfile::ROLL_SWEEP:
		return "roll_sweep";
	case IdentificationProfile::PITCH_SWEEP:
		return "pitch_sweep";
	case IdentificationProfile::YAW_SWEEP:
		return "yaw_sweep";
	case IdentificationProfile::DRAG_X:
		return "drag_x";
	case IdentificationProfile::DRAG_Y:
		return "drag_y";
	case IdentificationProfile::DRAG_Z:
		return "drag_z";
	case IdentificationProfile::MASS_VERTICAL:
		return "mass_vertical";
	case IdentificationProfile::MOTOR_STEP:
		return "motor_step";
	default:
		return "hover_thrust";
	}
}

const char *TrajectoryReader::identProfilePurpose(IdentificationProfile profile) const
{
	switch (profile) {
	case IdentificationProfile::HOVER_THRUST:
		return "vertical hover excitation for hover-thrust tracking";
	case IdentificationProfile::ROLL_SWEEP:
		return "lateral sweep for roll-axis inertia and coupling";
	case IdentificationProfile::PITCH_SWEEP:
		return "longitudinal sweep for pitch-axis inertia and coupling";
	case IdentificationProfile::YAW_SWEEP:
		return "yaw excitation for yaw inertia and moment balance";
	case IdentificationProfile::DRAG_X:
		return "forward-back motion for X-axis drag";
	case IdentificationProfile::DRAG_Y:
		return "side-to-side motion for Y-axis drag";
	case IdentificationProfile::DRAG_Z:
		return "vertical motion for Z-axis drag";
	case IdentificationProfile::MASS_VERTICAL:
		return "multi-frequency vertical motion for mass and thrust scale";
	case IdentificationProfile::MOTOR_STEP:
		return "step-like thrust sequence for motor time constants";
	default:
		return "system identification maneuver";
	}
}

const char *TrajectoryReader::campaignTypeToString(CampaignType type) const
{
	switch (type) {
	case CampaignType::IDENTIFICATION_ONLY:
		return "identification_only";

	case CampaignType::FULL_STACK:
		return "full_stack";

	case CampaignType::TRAJECTORY_ONLY:
		return "trajectory_only";

	case CampaignType::NONE:
	default:
		return "none";
	}
}

void TrajectoryReader::setCampaignStatusParam(int32_t status)
{
	param_t campaign_status = param_find("TRJ_CAMPAIGN_STA");
	if (campaign_status != PARAM_INVALID) {
		param_set(campaign_status, &status);
	}
}

bool TrajectoryReader::requestCampaignStart()
{
	if (_campaign_type == CampaignType::NONE) {
		PX4_WARN("Campaign start ignored: select TRJ_CAMPAIGN first");
		return false;
	}

	_campaign_start_requested = true;
	PX4_INFO("Campaign start requested: %s", campaignTypeToString(_campaign_type));
	return true;
}

void TrajectoryReader::stopCampaign(bool announce)
{
	const bool was_active = _campaign_active || _campaign_start_requested;

	_campaign_active = false;
	_campaign_start_requested = false;
	_campaign_stage = CampaignStage::IDLE;
	_campaign_item_index = 0;
	_campaign_anchor_settle_since = 0;
	_campaign_last_transition_time = 0;

	if (was_active) {
		const int32_t controller_type = 4;
		param_t controller_param = param_find("CST_POS_CTRL_TYP");
		if (controller_param != PARAM_INVALID) {
			param_set(controller_param, &controller_type);
		}
		_controller_type_cached = controller_type;
		setTrajectoryReaderMode(Mode::POSITION);
		const int32_t mode_value = static_cast<int32_t>(Mode::POSITION);
		param_t mode_param = param_find("TRJ_MODE_CMD");
		if (mode_param != PARAM_INVALID) {
			param_set(mode_param, &mode_value);
		}
		if (announce) {
			PX4_INFO("Campaign stopped");
		}
	}

	setCampaignStatusParam(was_active && announce ? 3 : 0);
}

bool TrajectoryReader::startCampaign(const matrix::Vector3f &current_pos, hrt_abstime now)
{
	size_t campaign_count = 0;
	const CampaignDefinitionItem *campaign_items = campaignDefinition(_campaign_type, campaign_count);

	if (!campaign_items || campaign_count == 0) {
		PX4_WARN("Campaign start ignored: no campaign selected");
		_campaign_start_requested = false;
		return false;
	}

	_campaign_anchor = current_pos;
	_campaign_yaw = _pos_yaw;
	_campaign_item_index = 0;
	_campaign_active = true;
	_campaign_start_requested = false;
	_campaign_stage = CampaignStage::RETURN_TO_ANCHOR;
	_campaign_anchor_settle_since = 0;
	_campaign_last_transition_time = now;
	setCampaignStatusParam(1);
	setTrajectoryAnchor(_campaign_anchor);
	const float anchor_values[3] = {
		_campaign_anchor(0),
		_campaign_anchor(1),
		_campaign_anchor(2)
	};
	param_t anchor_params[3] = {
		param_find("TRJ_ANCHOR_X"),
		param_find("TRJ_ANCHOR_Y"),
		param_find("TRJ_ANCHOR_Z"),
	};
	for (int i = 0; i < 3; ++i) {
		if (anchor_params[i] != PARAM_INVALID) {
			param_set(anchor_params[i], &anchor_values[i]);
		}
	}
	transitionCampaignToAnchor(current_pos, now, false);
	PX4_INFO("Campaign armed: %s (%u items)",
		 campaignTypeToString(_campaign_type),
		 static_cast<unsigned>(campaign_count));
	return true;
}

void TrajectoryReader::transitionCampaignToAnchor(const matrix::Vector3f &current_pos, hrt_abstime now, bool announce)
{
	const int32_t controller_type = 4;
	param_t controller_param = param_find("CST_POS_CTRL_TYP");
	if (controller_param != PARAM_INVALID) {
		param_set(controller_param, &controller_type);
	}
	_controller_type_cached = controller_type;
	setTrajectoryReaderMode(Mode::POSITION);
	setPositionModeRef(_campaign_anchor, _campaign_yaw, true);
	const int32_t mode_value = static_cast<int32_t>(Mode::POSITION);
	param_t mode_param = param_find("TRJ_MODE_CMD");
	if (mode_param != PARAM_INVALID) {
		param_set(mode_param, &mode_value);
	}
	_campaign_stage = CampaignStage::RETURN_TO_ANCHOR;
	_campaign_anchor_settle_since = ((current_pos - _campaign_anchor).norm() <= CAMPAIGN_RETURN_RADIUS_M) ? now : 0;
	_campaign_last_transition_time = now;

	if (announce) {
		PX4_INFO("Campaign returning to anchor [%.2f %.2f %.2f]",
			 (double)_campaign_anchor(0),
			 (double)_campaign_anchor(1),
			 (double)_campaign_anchor(2));
	}
}

bool TrajectoryReader::startCampaignSegment(size_t item_index, hrt_abstime now)
{
	size_t campaign_count = 0;
	const CampaignDefinitionItem *campaign_items = campaignDefinition(_campaign_type, campaign_count);

	if (!campaign_items || item_index >= campaign_count) {
		return false;
	}

	const CampaignDefinitionItem &item = campaign_items[item_index];
	setTrajectoryAnchor(_campaign_anchor);
	param_t controller_param = param_find("CST_POS_CTRL_TYP");

	if (item.mode == Mode::IDENTIFICATION) {
		const int32_t controller_type = 6;
		if (controller_param != PARAM_INVALID) {
			param_set(controller_param, &controller_type);
		}
		_controller_type_cached = controller_type;
		_ident_profile = item.ident_profile;
		const int32_t profile_value = static_cast<int32_t>(item.ident_profile);
		_param_ident_profile_cached = profile_value;
		param_t ident_param = param_find("TRJ_IDENT_PROF");
		if (ident_param != PARAM_INVALID) {
			param_set(ident_param, &profile_value);
		}
		setTrajectoryReaderMode(Mode::IDENTIFICATION);

	} else if (item.mode == Mode::TRAJECTORY) {
		const int32_t controller_type = 4;
		if (controller_param != PARAM_INVALID) {
			param_set(controller_param, &controller_type);
		}
		_controller_type_cached = controller_type;
		if (!setTrajectoryId(item.traj_id)) {
			PX4_ERR("Campaign failed to load trajectory %u", static_cast<unsigned>(item.traj_id));
			return false;
		}
		const int32_t traj_value = static_cast<int32_t>(item.traj_id);
		_param_traj_id_cached = traj_value;
		param_t traj_param = param_find("TRJ_ACTIVE_ID");
		if (traj_param != PARAM_INVALID) {
			param_set(traj_param, &traj_value);
		}
		setTrajectoryReaderMode(Mode::TRAJECTORY);
	}

	const int32_t mode_value = static_cast<int32_t>(item.mode);
	_param_mode_cmd_cached = mode_value;
	param_t mode_param = param_find("TRJ_MODE_CMD");
	if (mode_param != PARAM_INVALID) {
		param_set(mode_param, &mode_value);
	}

	_campaign_stage = CampaignStage::RUN_SEGMENT;
	_campaign_last_transition_time = now;
	_campaign_anchor_settle_since = 0;
	PX4_INFO("Campaign segment %u/%u started: %s",
		 static_cast<unsigned>(item_index + 1),
		 static_cast<unsigned>(campaign_count),
		 item.name);
	return true;
}

bool TrajectoryReader::campaignSegmentCompleted() const
{
	if (!_campaign_active || _campaign_stage != CampaignStage::RUN_SEGMENT) {
		return false;
	}

	if (_mode == Mode::IDENTIFICATION) {
		return _eof && _ident_log_fd < 0 && _tracking_log_fd < 0 && _pending_tracking_size == 0;
	}

	if (_mode == Mode::TRAJECTORY) {
		return _eof && _tracking_log_fd < 0 && _pending_tracking_size == 0;
	}

	return false;
}

void TrajectoryReader::updateCampaign(const matrix::Vector3f &current_pos, hrt_abstime now, bool in_offboard)
{
	if (!_campaign_active) {
		return;
	}

	if (!in_offboard) {
		PX4_WARN("Campaign aborted: OFFBOARD exited");
		stopCampaign(true);
		return;
	}

	size_t campaign_count = 0;
	const CampaignDefinitionItem *campaign_items = campaignDefinition(_campaign_type, campaign_count);

	if (!campaign_items || campaign_count == 0) {
		stopCampaign(true);
		return;
	}

	if (_campaign_stage == CampaignStage::RETURN_TO_ANCHOR) {
		const float anchor_distance = (current_pos - _campaign_anchor).norm();
		if (anchor_distance <= CAMPAIGN_RETURN_RADIUS_M) {
			if (_campaign_anchor_settle_since == 0) {
				_campaign_anchor_settle_since = now;
			}

			if (now - _campaign_anchor_settle_since >= CAMPAIGN_RETURN_SETTLE_US) {
				if (!startCampaignSegment(_campaign_item_index, now)) {
					PX4_ERR("Campaign failed to start segment %u", static_cast<unsigned>(_campaign_item_index));
					stopCampaign(true);
				}
			}
		} else {
			_campaign_anchor_settle_since = 0;
		}

		return;
	}

	if (_campaign_stage == CampaignStage::RUN_SEGMENT && campaignSegmentCompleted()) {
		PX4_INFO("Campaign segment completed");
		_campaign_item_index++;

		if (_campaign_item_index >= campaign_count) {
			_campaign_stage = CampaignStage::COMPLETE;
			setTrajectoryReaderMode(Mode::POSITION);
			setPositionModeRef(_campaign_anchor, _campaign_yaw, true);
			const int32_t mode_value = static_cast<int32_t>(Mode::POSITION);
			param_t mode_param = param_find("TRJ_MODE_CMD");
			if (mode_param != PARAM_INVALID) {
				param_set(mode_param, &mode_value);
			}
			_campaign_active = false;
			setCampaignStatusParam(2);
			PX4_INFO("Campaign completed: %s", campaignTypeToString(_campaign_type));
			return;
		}

		transitionCampaignToAnchor(current_pos, now, true);
	}
}

float TrajectoryReader::identificationDurationS(IdentificationProfile profile) const
{
	switch (profile) {
	case IdentificationProfile::HOVER_THRUST:
		return 26.0f;
	case IdentificationProfile::ROLL_SWEEP:
	case IdentificationProfile::PITCH_SWEEP:
		return 28.0f;
	case IdentificationProfile::YAW_SWEEP:
		return 24.0f;
	case IdentificationProfile::DRAG_X:
	case IdentificationProfile::DRAG_Y:
	case IdentificationProfile::DRAG_Z:
		return 30.0f;
	case IdentificationProfile::MASS_VERTICAL:
		return 36.0f;
	case IdentificationProfile::MOTOR_STEP:
		return 24.0f;
	default:
		return 26.0f;
	}
}


bool TrajectoryReader::openTrajectoryFile() {
    closeTrajectoryFile();

    char path[128];
    snprintf(path, sizeof(path),
         PX4_STORAGEDIR "/trajectories/id_%u.traj",
         _traj_id);
    
    struct stat st{};

    if (stat(path, &st) != 0) {
        PX4_ERR("Trajectory file does not exist: %s", path);
        PX4_INFO("For hardware/HITL, copy id_100..104.traj into " PX4_STORAGEDIR "/trajectories/");
        return false;
    }

    if (st.st_size == 0 || (st.st_size % sizeof(TrajSample) != 0)) {
        PX4_ERR("Invalid Trajectory file");
        return false;
    }

    _fd = open(path, O_RDONLY);

    if (_fd < 0) {
        PX4_ERR("Failed opening Trajectory file");
        return false;
    }

    _num_samples = st.st_size / sizeof(TrajSample);
    _cursor = 0;
    _eof = false;

    PX4_INFO("Opened trajectory (%lu samples)", (long unsigned int)_num_samples);
    return true;
}

void TrajectoryReader::closeTrajectoryFile() {
    // if file is open
    if (_fd >= 0) {
        close(_fd);
        _fd = -1;
    }

	_num_samples = 0;
	_cursor = 0;
	_eof = true;
}

bool TrajectoryReader::storageRootAvailable() const
{
	struct stat st{};

	if (stat(PX4_STORAGEDIR, &st) != 0) {
		PX4_ERR("Storage root unavailable: %s (%d)", PX4_STORAGEDIR, errno);
		return false;
	}

	if (!S_ISDIR(st.st_mode)) {
		PX4_ERR("Storage root is not a directory: %s", PX4_STORAGEDIR);
		return false;
	}

	return true;
}

void TrajectoryReader::clearTrackingLogFailure()
{
	if (_tracking_log_failed_latched) {
		PX4_INFO("Clearing previous tracking log failure state");
	}

	_tracking_log_failed_latched = false;
}

bool TrajectoryReader::startTrajectoryTrackingLog(hrt_abstime setpoint_timestamp)
{
	if (_tracking_log_fd >= 0) {
		return true;
	}

	if (_tracking_log_failed_latched) {
		PX4_WARN("Retrying tracking log after a previous failure");
		_tracking_log_failed_latched = false;
	}

	if (!storageRootAvailable()) {
		_tracking_log_failed_latched = true;
		_tracking_log_fail_count++;
		return false;
	}

	constexpr mode_t dir_mode = S_IRWXU | S_IRWXG | S_IRWXO;
	constexpr char log_dir[] = PX4_STORAGEDIR "/tracking_logs";

	if (mkdir(log_dir, dir_mode) != 0 && errno != EEXIST) {
		PX4_ERR("Failed to create tracking log dir: %s (%d)", log_dir, errno);
		_tracking_log_failed_latched = true;
		_tracking_log_fail_count++;
		if (_mode == Mode::TRAJECTORY) {
			setTrajectoryReaderMode(Mode::POSITION);
		}
		return false;
	}

	const uint16_t next_run_counter = static_cast<uint16_t>(_tracking_log_run_counter + 1U);

	char path[192];
	if (_mode == Mode::IDENTIFICATION) {
		snprintf(path, sizeof(path),
			 PX4_STORAGEDIR "/tracking_logs/%s_r%u_%llx.csv",
			 identProfileToString(_ident_profile),
			 static_cast<unsigned>(next_run_counter),
			 static_cast<unsigned long long>(setpoint_timestamp));
	} else {
		snprintf(path, sizeof(path),
			 PX4_STORAGEDIR "/tracking_logs/t%ur%u_%llx.csv",
			 _traj_id,
			 static_cast<unsigned>(next_run_counter),
			 static_cast<unsigned long long>(setpoint_timestamp));
	}

	_tracking_log_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

	if (_tracking_log_fd < 0) {
		PX4_INFO("Tracking log open failed: %s (%d)", path, errno);
		_tracking_log_failed_latched = true;
		_tracking_log_fail_count++;
		if (_mode == Mode::TRAJECTORY) {
			setTrajectoryReaderMode(Mode::POSITION);
		}
		return false;
	}

	const char *header = "timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n";
	const ssize_t header_len = strlen(header);

	if (write(_tracking_log_fd, header, header_len) != header_len) {
		PX4_ERR("Failed writing tracking log header");
		close(_tracking_log_fd);
		_tracking_log_fd = -1;
		_tracking_log_failed_latched = true;
		_tracking_log_fail_count++;
		if (_mode == Mode::TRAJECTORY) {
			setTrajectoryReaderMode(Mode::POSITION);
		}
		return false;
	}

	_tracking_log_samples_since_sync = 0;
	syncTrackingLogIfNeeded(true);
	_tracking_log_run_counter = next_run_counter;
	PX4_INFO("Tracking log started: %s", path);
	return true;
}

void TrajectoryReader::clearPendingTrackingSamples()
{
	_pending_tracking_head = 0;
	_pending_tracking_size = 0;
}

void TrajectoryReader::stopTrajectoryTrackingLog()
{
	if (_tracking_log_fd >= 0) {
		syncTrackingLogIfNeeded(true);
		close(_tracking_log_fd);
		_tracking_log_fd = -1;
	}

	_tracking_log_samples_since_sync = 0;
	clearPendingTrackingSamples();
	_start_new_tracking_log = false;
	_finalize_tracking_log = false;
}

void TrajectoryReader::resetIdentificationState()
{
	_ident_origin_valid = false;
	_ident_origin = {};
	_ident_start_time = 0;
	_ident_started = false;
	_ident_finalize_log = false;
	_ident_start_announced = false;
	_ident_completion_announced = false;
	_cursor = 0;
	_num_samples = 0;
	_eof = false;
}

bool TrajectoryReader::startIdentificationLog(hrt_abstime setpoint_timestamp)
{
	if (_ident_log_fd >= 0) {
		return true;
	}

	if (!storageRootAvailable()) {
		return false;
	}

	constexpr mode_t dir_mode = S_IRWXU | S_IRWXG | S_IRWXO;
	constexpr char log_dir[] = PX4_STORAGEDIR "/identification_logs";

	if (mkdir(log_dir, dir_mode) != 0 && errno != EEXIST) {
		PX4_ERR("Failed to create identification log dir: %s (%d)", log_dir, errno);
		return false;
	}

	const uint16_t next_run_counter = static_cast<uint16_t>(_ident_log_run_counter + 1U);
	char path[192];
	snprintf(path, sizeof(path),
		 PX4_STORAGEDIR "/identification_logs/%s_r%u_%llx.csv",
		 identProfileToString(_ident_profile),
		 static_cast<unsigned>(next_run_counter),
		 static_cast<unsigned long long>(setpoint_timestamp));

	_ident_log_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);
	if (_ident_log_fd < 0) {
		PX4_ERR("Failed to open identification log: %s (%d)", path, errno);
		return false;
	}

	const char *header =
		"timestamp_us,profile,ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz,ref_ax,ref_ay,ref_az,"
		"ref_yaw,ref_yawspeed,pos_x,pos_y,pos_z,vx,vy,vz,ax,ay,az,roll,pitch,yaw,p,q,r,"
		"att_sp_q0,att_sp_q1,att_sp_q2,att_sp_q3,att_sp_thrust_x,att_sp_thrust_y,att_sp_thrust_z,"
		"rate_sp_roll,rate_sp_pitch,rate_sp_yaw,rate_sp_thrust_x,rate_sp_thrust_y,rate_sp_thrust_z,"
		"motor_0,motor_1,motor_2,motor_3,"
		"esc_0_rpm,esc_1_rpm,esc_2_rpm,esc_3_rpm,"
		"esc_0_voltage,esc_1_voltage,esc_2_voltage,esc_3_voltage,"
		"esc_0_current,esc_1_current,esc_2_current,esc_3_current,"
		"hover_thrust,controller\n";
	const ssize_t header_len = strlen(header);
	if (write(_ident_log_fd, header, header_len) != header_len) {
		PX4_ERR("Failed writing identification log header");
		close(_ident_log_fd);
		_ident_log_fd = -1;
		return false;
	}

	_ident_log_samples_since_sync = 0;
	syncIdentificationLogIfNeeded(true);
	_ident_log_run_counter = next_run_counter;
	PX4_INFO("Identification log started: %s", path);
	return true;
}

void TrajectoryReader::stopIdentificationLog()
{
	if (_ident_log_fd >= 0) {
		syncIdentificationLogIfNeeded(true);
		close(_ident_log_fd);
		_ident_log_fd = -1;
	}

	_ident_log_samples_since_sync = 0;
}

void TrajectoryReader::syncTrackingLogIfNeeded(bool force)
{
	if (_tracking_log_fd < 0) {
		return;
	}

	if (force || _tracking_log_samples_since_sync >= LOG_FSYNC_INTERVAL_SAMPLES) {
		(void)fsync(_tracking_log_fd);
		_tracking_log_samples_since_sync = 0;
	}
}

void TrajectoryReader::syncIdentificationLogIfNeeded(bool force)
{
	if (_ident_log_fd < 0) {
		return;
	}

	if (force || _ident_log_samples_since_sync >= LOG_FSYNC_INTERVAL_SAMPLES) {
		(void)fsync(_ident_log_fd);
		_ident_log_samples_since_sync = 0;
	}
}

matrix::Vector3f TrajectoryReader::identificationRelativePosition(IdentificationProfile profile, float t_s) const
{
	const float two_pi = 2.0f * M_PI_F;
	switch (profile) {
	case IdentificationProfile::HOVER_THRUST:
		return matrix::Vector3f(
			0.0f,
			0.0f,
			0.35f * sinf(two_pi * 0.12f * t_s) + 0.18f * sinf(two_pi * 0.27f * t_s));
	case IdentificationProfile::ROLL_SWEEP:
		return matrix::Vector3f(
			0.0f,
			0.60f * sinf(two_pi * 0.09f * t_s)
				+ 0.28f * sinf(two_pi * 0.19f * t_s)
				+ 0.12f * sinf(two_pi * 0.31f * t_s),
			0.0f);
	case IdentificationProfile::PITCH_SWEEP:
		return matrix::Vector3f(
			0.60f * sinf(two_pi * 0.09f * t_s)
				+ 0.28f * sinf(two_pi * 0.19f * t_s)
				+ 0.12f * sinf(two_pi * 0.31f * t_s),
			0.0f,
			0.0f);
	case IdentificationProfile::YAW_SWEEP:
		return matrix::Vector3f(0.0f, 0.0f, 0.0f);
	case IdentificationProfile::DRAG_X:
		return matrix::Vector3f(
			1.20f * sinf(two_pi * 0.08f * t_s),
			0.0f,
			0.0f);
	case IdentificationProfile::DRAG_Y:
		return matrix::Vector3f(
			0.0f,
			1.20f * sinf(two_pi * 0.08f * t_s),
			0.0f);
	case IdentificationProfile::DRAG_Z:
		return matrix::Vector3f(
			0.0f,
			0.0f,
			0.50f * sinf(two_pi * 0.09f * t_s));
	case IdentificationProfile::MASS_VERTICAL:
		return matrix::Vector3f(
			0.0f,
			0.0f,
			0.55f * sinf(two_pi * 0.08f * t_s)
				+ 0.35f * sinf(two_pi * 0.17f * t_s)
				+ 0.18f * sinf(two_pi * 0.29f * t_s)
				+ 0.10f * sinf(two_pi * 0.41f * t_s));
	case IdentificationProfile::MOTOR_STEP: {
		const float segment = floorf(t_s / 2.5f);
		const float level = math::constrain(segment, 0.0f, 7.0f);
		const float sequence[] = {0.00f, 0.06f, -0.02f, 0.11f, -0.04f, 0.16f, -0.06f, 0.00f};
		const int idx = math::constrain(static_cast<int>(level), 0, 7);
		return matrix::Vector3f(0.0f, 0.0f, sequence[idx]);
	}
	default:
		return matrix::Vector3f();
	}
}

float TrajectoryReader::identificationRelativeYaw(IdentificationProfile profile, float t_s) const
{
	if (profile == IdentificationProfile::YAW_SWEEP) {
		return 0.55f * sinf(2.0f * M_PI_F * 0.07f * t_s)
			+ 0.22f * sinf(2.0f * M_PI_F * 0.15f * t_s);
	}
	return 0.0f;
}

float TrajectoryReader::identificationRelativeYawRate(IdentificationProfile profile, float t_s) const
{
	if (profile == IdentificationProfile::YAW_SWEEP) {
		return 0.55f * 2.0f * M_PI_F * 0.07f * cosf(2.0f * M_PI_F * 0.07f * t_s)
			+ 0.22f * 2.0f * M_PI_F * 0.15f * cosf(2.0f * M_PI_F * 0.15f * t_s);
	}
	return 0.0f;
}

void TrajectoryReader::identificationKinematics(
	IdentificationProfile profile,
	float t_s,
	matrix::Vector3f &position,
	matrix::Vector3f &velocity,
	matrix::Vector3f &acceleration,
	float &yaw,
	float &yawspeed) const
{
	const float dt = 0.02f;
	const matrix::Vector3f pm = identificationRelativePosition(profile, math::max(0.0f, t_s - dt));
	const matrix::Vector3f p0 = identificationRelativePosition(profile, t_s);
	const matrix::Vector3f pp = identificationRelativePosition(profile, t_s + dt);

	position = p0;
	velocity = (pp - pm) / (2.0f * dt);
	acceleration = (pp - 2.0f * p0 + pm) / squaref(dt);

	const float ym = identificationRelativeYaw(profile, math::max(0.0f, t_s - dt));
	const float y0 = identificationRelativeYaw(profile, t_s);
	const float yp = identificationRelativeYaw(profile, t_s + dt);
	yaw = y0;
	yawspeed = identificationRelativeYawRate(profile, t_s);
	if (!PX4_ISFINITE(yawspeed)) {
		yawspeed = (yp - ym) / (2.0f * dt);
	}
}

void TrajectoryReader::fillIdentificationBuffer(const matrix::Vector3f &current_pos, hrt_abstime now)
{
	if (!_ident_origin_valid) {
		_ident_origin = _traj_anchor_valid ? _traj_anchor : current_pos;
		_ident_origin_valid = true;
	}

	if (!_ident_started) {
		_ident_start_time = now;
		_ident_started = true;
		_start_new_tracking_log = true;
		_ident_finalize_log = false;
		_eof = false;
		_cursor = 0;
		const float duration_s = identificationDurationS(_ident_profile);
		_num_samples = static_cast<uint32_t>(duration_s / 0.02f);
		if (!_ident_start_announced) {
			PX4_INFO("Identification maneuver started: %s", identProfileToString(_ident_profile));
			PX4_INFO("  Purpose: %s", identProfilePurpose(_ident_profile));
			PX4_INFO("  Estimated duration: %.1f s", (double)duration_s);
			PX4_INFO("  After completion: hold the final reference until the next profile, mode change, or landing");
			_ident_start_announced = true;
		}
	}

	const float elapsed_s = math::max(0.0f, (now - _ident_start_time) * 1e-6f);
	const float duration_s = identificationDurationS(_ident_profile);
	const bool finished = elapsed_s >= duration_s;

	for (size_t i = 0; i < MAX_HORIZON; ++i) {
		const float sample_time_s = math::min(duration_s, elapsed_s + static_cast<float>(i) * 0.02f);
		matrix::Vector3f rel_pos{};
		matrix::Vector3f rel_vel{};
		matrix::Vector3f rel_acc{};
		float rel_yaw = 0.0f;
		float rel_yawspeed = 0.0f;
		identificationKinematics(_ident_profile, sample_time_s, rel_pos, rel_vel, rel_acc, rel_yaw, rel_yawspeed);

		const matrix::Vector3f pos = _ident_origin + rel_pos;
		_buffer[i].px = pos(0);
		_buffer[i].py = pos(1);
		_buffer[i].pz = pos(2);
		_buffer[i].vx = rel_vel(0);
		_buffer[i].vy = rel_vel(1);
		_buffer[i].vz = rel_vel(2);
		_buffer[i].ax = rel_acc(0);
		_buffer[i].ay = rel_acc(1);
		_buffer[i].az = rel_acc(2);
		_buffer[i].yaw = rel_yaw;
		_buffer[i].yawspeed = rel_yawspeed;
	}

	_buffer_len = MAX_HORIZON;
	_cursor = math::min(_num_samples, static_cast<uint32_t>(elapsed_s / 0.02f));
	_eof = finished;
	if (finished) {
		_ident_finalize_log = true;
		if (!_ident_completion_announced) {
			PX4_INFO("Identification maneuver completed: %s", identProfileToString(_ident_profile));
			PX4_INFO("  Logged duration: %.1f s", (double)duration_s);
			PX4_INFO("  Vehicle state: holding the final reference while waiting for the next command");
			_ident_completion_announced = true;
		}
	}
}

void TrajectoryReader::writeIdentificationLogSample(const matrix::Vector3f &current_pos, hrt_abstime now)
{
	if (_ident_log_fd < 0 || _buffer_len == 0) {
		return;
	}

	vehicle_attitude_s attitude{};
	_attitude_sub.copy(&attitude);
	vehicle_angular_velocity_s angular_velocity{};
	_angular_velocity_sub.copy(&angular_velocity);
	vehicle_attitude_setpoint_s attitude_sp{};
	_attitude_sp_sub.copy(&attitude_sp);
	vehicle_rates_setpoint_s rates_sp{};
	_rates_sp_sub.copy(&rates_sp);
	actuator_motors_s actuator_motors{};
	_actuator_motors_sub.copy(&actuator_motors);
	esc_status_s esc_status{};
	_esc_status_sub.copy(&esc_status);
	hover_thrust_estimate_s hover{};
	_hover_thrust_sub.copy(&hover);
	vehicle_local_position_s local_pos{};
	_local_pos_sub.copy(&local_pos);

	double esc_rpm[4] = {NAN, NAN, NAN, NAN};
	double esc_voltage[4] = {NAN, NAN, NAN, NAN};
	double esc_current[4] = {NAN, NAN, NAN, NAN};
	const uint8_t esc_count = math::min<uint8_t>(4, esc_status.esc_count);
	for (uint8_t i = 0; i < esc_count; ++i) {
		esc_rpm[i] = static_cast<double>(esc_status.esc[i].esc_rpm);
		esc_voltage[i] = static_cast<double>(esc_status.esc[i].esc_voltage);
		esc_current[i] = static_cast<double>(esc_status.esc[i].esc_current);
	}

	float roll = NAN;
	float pitch = NAN;
	float yaw = NAN;
	if (PX4_ISFINITE(attitude.q[0]) && PX4_ISFINITE(attitude.q[1]) &&
	    PX4_ISFINITE(attitude.q[2]) && PX4_ISFINITE(attitude.q[3])) {
		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		roll = euler.phi();
		pitch = euler.theta();
		yaw = euler.psi();
	}

	const auto write_chunk = [&](const char *fmt, auto... values) -> bool {
		return dprintf(_ident_log_fd, fmt, values...) >= 0;
	};

	if (!write_chunk(
		"%llu,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
		"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
		static_cast<unsigned long long>(now),
		identProfileToString(_ident_profile),
		static_cast<double>(_buffer[0].px),
		static_cast<double>(_buffer[0].py),
		static_cast<double>(_buffer[0].pz),
		static_cast<double>(_buffer[0].vx),
		static_cast<double>(_buffer[0].vy),
		static_cast<double>(_buffer[0].vz),
		static_cast<double>(_buffer[0].ax),
		static_cast<double>(_buffer[0].ay),
		static_cast<double>(_buffer[0].az),
		static_cast<double>(_buffer[0].yaw),
		static_cast<double>(_buffer[0].yawspeed),
		static_cast<double>(current_pos(0)),
		static_cast<double>(current_pos(1)),
		static_cast<double>(current_pos(2)),
		static_cast<double>(local_pos.vx),
		static_cast<double>(local_pos.vy),
		static_cast<double>(local_pos.vz),
		static_cast<double>(local_pos.ax),
		static_cast<double>(local_pos.ay),
		static_cast<double>(local_pos.az),
		static_cast<double>(roll),
		static_cast<double>(pitch),
		static_cast<double>(yaw),
		static_cast<double>(angular_velocity.xyz[0]),
		static_cast<double>(angular_velocity.xyz[1]),
		static_cast<double>(angular_velocity.xyz[2]))
		|| !write_chunk(
		"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
		static_cast<double>(attitude_sp.q_d[0]),
		static_cast<double>(attitude_sp.q_d[1]),
		static_cast<double>(attitude_sp.q_d[2]),
		static_cast<double>(attitude_sp.q_d[3]),
		static_cast<double>(attitude_sp.thrust_body[0]),
		static_cast<double>(attitude_sp.thrust_body[1]),
		static_cast<double>(attitude_sp.thrust_body[2]),
		static_cast<double>(rates_sp.roll),
		static_cast<double>(rates_sp.pitch),
		static_cast<double>(rates_sp.yaw),
		static_cast<double>(rates_sp.thrust_body[0]),
		static_cast<double>(rates_sp.thrust_body[1]),
		static_cast<double>(rates_sp.thrust_body[2]),
		static_cast<double>(actuator_motors.control[0]),
		static_cast<double>(actuator_motors.control[1]),
		static_cast<double>(actuator_motors.control[2]))
		|| !write_chunk(
		"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
		static_cast<double>(actuator_motors.control[3]),
		esc_rpm[0],
		esc_rpm[1],
		esc_rpm[2],
		esc_rpm[3],
		esc_voltage[0],
		esc_voltage[1],
		esc_voltage[2],
		esc_voltage[3],
		esc_current[0],
		esc_current[1],
		esc_current[2],
		esc_current[3],
		static_cast<double>(hover.hover_thrust),
		controllerTypeToString(_controller_type_cached))) {
		PX4_ERR("Failed writing identification log row");
		stopIdentificationLog();
		return;
	}

	if (_ident_log_samples_since_sync < LOG_FSYNC_INTERVAL_SAMPLES) {
		_ident_log_samples_since_sync++;
	}

	syncIdentificationLogIfNeeded(false);
}

void TrajectoryReader::updateRcSelections()
{
	if (_rc_select_enabled == 0 || _rc_selector_channel < 1 || _rc_selector_channel > 6) {
		_rc_selected_index = -1;
		return;
	}

	manual_control_setpoint_s manual_control{};
	if (!_manual_control_sub.update(&manual_control) || !manual_control.valid) {
		return;
	}

	const float aux_value = readAuxChannel(manual_control, _rc_selector_channel);
	const int slot_count = (_mode == Mode::IDENTIFICATION)
		? 9
		: math::max<int32_t>(1, _rc_selector_max_traj_id + 1);
	const int selection = quantizeAuxSelection(aux_value, slot_count);
	if (selection < 0 || selection == _rc_selected_index) {
		return;
	}

	_rc_selected_index = selection;
	if (_mode == Mode::IDENTIFICATION) {
		_ident_profile = static_cast<IdentificationProfile>(selection);
		param_t ident_param = param_find("TRJ_IDENT_PROF");
		const int32_t profile_value = selection;
		if (ident_param != PARAM_INVALID) {
			param_set(ident_param, &profile_value);
		}
		stopTrajectoryTrackingLog();
		stopIdentificationLog();
		resetIdentificationState();
		_start_new_tracking_log = true;
	} else {
		if (setTrajectoryId(static_cast<uint8_t>(selection))) {
			const int32_t traj_value = selection;
			param_t traj_param = param_find("TRJ_ACTIVE_ID");
			if (traj_param != PARAM_INVALID) {
				param_set(traj_param, &traj_value);
			}
		}
	}
}

void TrajectoryReader::queueTrackingLogSample(const matrix::Vector3f &ref_first_pos, hrt_abstime setpoint_timestamp)
{
	if (_pending_tracking_size >= MAX_PENDING_TRACKING_SAMPLES) {
		_pending_tracking_head = (_pending_tracking_head + 1) % MAX_PENDING_TRACKING_SAMPLES;
		_pending_tracking_size--;
		PX4_WARN("Tracking log queue full, dropping oldest sample");
	}

	const size_t write_index = (_pending_tracking_head + _pending_tracking_size) % MAX_PENDING_TRACKING_SAMPLES;

	_pending_tracking_samples[write_index].setpoint_timestamp = setpoint_timestamp;
	_pending_tracking_samples[write_index].ref_first_pos = ref_first_pos;
	_pending_tracking_samples[write_index].controller_type = _controller_type_cached;
	_pending_tracking_size++;
}

void TrajectoryReader::flushTrackingLogSamples(const matrix::Vector3f &current_pos, hrt_abstime now)
{
	if (_tracking_log_fd < 0) {
		return;
	}

	while (_pending_tracking_size > 0) {
		const PendingTrackingSample &sample = _pending_tracking_samples[_pending_tracking_head];

		if (now < sample.setpoint_timestamp + TRACKING_COMPARISON_DELAY_US) {
			break;
		}

		char line[256];
		const int line_len = snprintf(line, sizeof(line),
				      "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
				      static_cast<unsigned long long>(now),
				      static_cast<double>(sample.ref_first_pos(0)),
				      static_cast<double>(sample.ref_first_pos(1)),
				      static_cast<double>(sample.ref_first_pos(2)),
				      static_cast<double>(current_pos(0)),
				      static_cast<double>(current_pos(1)),
				      static_cast<double>(current_pos(2)),
				      controllerTypeToString(sample.controller_type));

		if (line_len <= 0 || line_len >= static_cast<int>(sizeof(line))) {
			PX4_ERR("Failed formatting tracking log row");
			stopTrajectoryTrackingLog();
			return;
		}

		if (write(_tracking_log_fd, line, line_len) != line_len) {
			PX4_ERR("Failed writing tracking log row");
			stopTrajectoryTrackingLog();
			return;
		}

		if (_tracking_log_samples_since_sync < LOG_FSYNC_INTERVAL_SAMPLES) {
			_tracking_log_samples_since_sync++;
		}

		syncTrackingLogIfNeeded(false);

		_pending_tracking_head = (_pending_tracking_head + 1) % MAX_PENDING_TRACKING_SAMPLES;
		_pending_tracking_size--;
	}

	if (_finalize_tracking_log && _pending_tracking_size == 0) {
		if (_mode == Mode::IDENTIFICATION) {
			PX4_INFO("Tracking log completed: %s", identProfileToString(_ident_profile));
		} else {
			PX4_INFO("Trajectory EOF reached, tracking log closed");
		}
		stopTrajectoryTrackingLog();
	}
}

bool TrajectoryReader::loadNextHorizon() {
    // if file not open
    if (_fd < 0) {
        return false;
    }

    // first fill
    if (_buffer_len == 0 && !_eof) {
        for (size_t i = 0; i < MAX_HORIZON; ++i) {
            ssize_t bytes = read(_fd, &_buffer[i], sizeof(TrajSample));

            if (bytes == 0) {
                _eof = true;
                break;
            }

            if (bytes != sizeof(TrajSample)) {
                PX4_ERR("Corrupted Trajectory File.");
                return false;
            }

            _buffer_len++;
            _cursor++;
        }

        // if file shorter than single horizon -> pad
        if (_buffer_len == 0) {
            PX4_ERR("Trajectory file contains no valid samples.");
            return false;
        }

        const TrajSample &last = _buffer[_buffer_len - 1];

        for (size_t i = _buffer_len; i < MAX_HORIZON; ++i) {
            _buffer[i] = last;
        }

        _buffer_len = MAX_HORIZON;
        return true;
    }

    // sliding horizon
    for (size_t i = 0; i < MAX_HORIZON - 1; ++i) {
        _buffer[i] = _buffer[i + 1];
    }

    // append last sample
    if (!_eof) {
        ssize_t bytes = read(_fd, &_buffer[MAX_HORIZON - 1], sizeof(TrajSample));

        if (bytes == 0) {
            _eof = true;
        } else if (bytes != sizeof(TrajSample)) {
            PX4_ERR("Corrupted Trajectory File.");
            return false;
        } else {
            _cursor++;
        }
    }

    // hold last if -> eof
    if (_eof) {
        _buffer[MAX_HORIZON - 1] = _buffer[MAX_HORIZON - 2];
    }

    _buffer_len = MAX_HORIZON;
    return true;
}

void TrajectoryReader::publishSetpoint(const matrix::Vector3f &current_pos) {
    if (_buffer_len < MAX_HORIZON) {
        return;
    }

    multi_trajectory_setpoint_s ref{};

    ref.timestamp = hrt_absolute_time();
    ref.horizon_length = MAX_HORIZON;
    ref.valid = true;

    for (size_t i = 0; i < MAX_HORIZON; ++i) {
        const TrajSample &s = _buffer[i];

        ref.position_x[i] = s.px;
        ref.position_y[i] = s.py;
        ref.position_z[i] = s.pz;

		if (_mode == Mode::TRAJECTORY && _traj_offset_valid) {
			ref.position_x[i] += _traj_offset(0);
			ref.position_y[i] += _traj_offset(1);
			ref.position_z[i] += _traj_offset(2);
		}

        ref.velocity_x[i] = s.vx;
        ref.velocity_y[i] = s.vy;
        ref.velocity_z[i] = s.vz;

        ref.accel_x[i] = s.ax;
        ref.accel_y[i] = s.ay;
        ref.accel_z[i] = s.az;

        ref.yaw[i] = s.yaw;
        ref.yawspeed[i] = s.yawspeed;
    }

	_pub.publish(ref);

	if (_mode == Mode::TRAJECTORY || _mode == Mode::IDENTIFICATION) {
		if (_start_new_tracking_log && !_finalize_tracking_log) {
			if (startTrajectoryTrackingLog(ref.timestamp)) {
				_start_new_tracking_log = false;
			}
		}

		if (_mode == Mode::IDENTIFICATION && _ident_log_fd < 0 && !_ident_finalize_log) {
			startIdentificationLog(ref.timestamp);
		}

		const matrix::Vector3f first_ref_pos{
			ref.position_x[0],
			ref.position_y[0],
			ref.position_z[0]
		};

		if (!_finalize_tracking_log && _tracking_log_fd >= 0) {
			queueTrackingLogSample(first_ref_pos, ref.timestamp);
		}

		flushTrackingLogSamples(current_pos, hrt_absolute_time());

		if (_mode == Mode::IDENTIFICATION) {
			writeIdentificationLogSample(current_pos, hrt_absolute_time());
			if (_ident_finalize_log && _eof && _ident_log_fd >= 0) {
				PX4_INFO("Identification log completed: %s", identProfileToString(_ident_profile));
				stopIdentificationLog();
			}
		}
	}
}

void TrajectoryReader::publishHoldPositionSetpoint(const matrix::Vector3f &current_pos)
{
	const float dt = 0.02f;

	if (!_pos_ref_absolute && (_need_pos_offset || !_pos_offset_valid)) {
		_pos_offset = current_pos;
		_pos_offset_valid = true;
		_need_pos_offset = false;
	}

	const matrix::Vector3f pos_target = _pos_ref_absolute
		? _pos_target
		: (_pos_offset_valid ? (_pos_offset + _pos_target) : current_pos);

	goto_setpoint_s goto_sp{};
	goto_sp.position[0] = pos_target(0);
	goto_sp.position[1] = pos_target(1);
	goto_sp.position[2] = pos_target(2);
	goto_sp.flag_control_heading = true;
	goto_sp.heading = _pos_yaw;

	setPositionSmootherLimits(goto_sp);
	setHeadingSmootherLimits(goto_sp);

	matrix::Vector3f pos = current_pos;
	auto pos_tmp = _position_smoothing;
	auto yaw_tmp = _heading_smoothing;

	for (size_t i = 0; i < MAX_HORIZON; i++) {
		PositionSmoothing::PositionSmoothingSetpoints out{};
		pos_tmp.generateSetpoints(pos, pos_target, {}, dt, false, out);
		pos = out.position;
		yaw_tmp.update(_pos_yaw, dt);

		_buffer[i].px = out.position(0);
		_buffer[i].py = out.position(1);
		_buffer[i].pz = out.position(2);
		_buffer[i].vx = out.velocity(0);
		_buffer[i].vy = out.velocity(1);
		_buffer[i].vz = out.velocity(2);
		_buffer[i].ax = out.acceleration(0);
		_buffer[i].ay = out.acceleration(1);
		_buffer[i].az = out.acceleration(2);
		_buffer[i].yaw = yaw_tmp.getSmoothedHeading();
		_buffer[i].yawspeed = yaw_tmp.getSmoothedHeadingRate();

		if (i == 0) {
			_position_smoothing = pos_tmp;
			_heading_smoothing = yaw_tmp;
		}
	}

	_buffer_len = MAX_HORIZON;

	multi_trajectory_setpoint_s ref{};
	ref.timestamp = hrt_absolute_time();
	ref.horizon_length = MAX_HORIZON;
	ref.valid = true;

	for (size_t i = 0; i < MAX_HORIZON; ++i) {
		const TrajSample &s = _buffer[i];
		ref.position_x[i] = s.px;
		ref.position_y[i] = s.py;
		ref.position_z[i] = s.pz;
		ref.velocity_x[i] = s.vx;
		ref.velocity_y[i] = s.vy;
		ref.velocity_z[i] = s.vz;
		ref.accel_x[i] = s.ax;
		ref.accel_y[i] = s.ay;
		ref.accel_z[i] = s.az;
		ref.yaw[i] = s.yaw;
		ref.yawspeed[i] = s.yawspeed;
	}

	_pub.publish(ref);
}

void TrajectoryReader::Run()
{
	if (should_exit()) {
		stopTrajectoryTrackingLog();
		closeTrajectoryFile();
		exit_and_cleanup(desc);
		return;
	}

	// only update parameters when params are updated
	if (_param_update_sub.updated()) {
		_param_update_sub.copy(nullptr);
		parametersUpdate();
		updateControllerTypeCache();
	}

	updateRcSelections();

	_vehicle_status_sub.update(&_status);
	// deceted offboard entry
	const bool entered_offboard =
		(_prev_nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) &&
		(_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	_prev_nav_state = _status.nav_state;
	const bool armed = (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool in_offboard = (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	if (!armed) {
		if ((_mode == Mode::TRAJECTORY || _mode == Mode::IDENTIFICATION) && (_tracking_log_fd >= 0 || _pending_tracking_size > 0)) {
			stopTrajectoryTrackingLog();
			_start_new_tracking_log = true;
		}

		if (_mode == Mode::IDENTIFICATION) {
			stopIdentificationLog();
			resetIdentificationState();
		}

		return;
	}

	vehicle_local_position_s vehicle_local_position{};
	if (!_local_pos_sub.update(&vehicle_local_position)) {
		return;
	}

	const matrix::Vector3f current_pos{
		vehicle_local_position.x,
		vehicle_local_position.y,
		vehicle_local_position.z
	};
	const hrt_abstime now = hrt_absolute_time();

	// Keep a hold reference streaming while armed so OFFBOARD can latch cleanly,
	// even if the operator has already selected TRAJECTORY or IDENTIFICATION mode.
	if ((_mode == Mode::TRAJECTORY || _mode == Mode::IDENTIFICATION) && !in_offboard) {
		if (_campaign_active) {
			PX4_WARN("Campaign aborted before OFFBOARD became active");
			stopCampaign(true);
		}

		if ((_tracking_log_fd >= 0 || _pending_tracking_size > 0)) {
			stopTrajectoryTrackingLog();
			_start_new_tracking_log = true;
		}

		if (_mode == Mode::IDENTIFICATION) {
			stopIdentificationLog();
			resetIdentificationState();
		}

		if (_need_reset) {
			resetSmoothers(current_pos, _pos_yaw);
		}

		publishHoldPositionSetpoint(current_pos);
		return;
	}

	if (_campaign_start_requested && !_campaign_active && in_offboard) {
		startCampaign(current_pos, now);
	}

	if (entered_offboard) {
		// Preserve the selected workflow across OFFBOARD entry.
		// We only reset the state that must be recaptured at the mode boundary.
		if (_mode == Mode::POSITION) {
			const auto reset = position_mode_offboard_entry_reset(_pos_ref_absolute);
			_need_pos_offset = reset.need_pos_offset;
			_pos_offset_valid = reset.pos_offset_valid;
			_pos_ref_absolute = reset.preserve_absolute_reference;

		} else if (_mode == Mode::TRAJECTORY) {
			stopTrajectoryTrackingLog();
			closeTrajectoryFile();
			_buffer_len = 0;
			_eof = false;
			_finalize_tracking_log = false;
			_traj_offset_valid = false;
			_need_traj_offset = true;
			_start_new_tracking_log = true;

		} else if (_mode == Mode::IDENTIFICATION) {
			stopTrajectoryTrackingLog();
			stopIdentificationLog();
			_finalize_tracking_log = false;
			_start_new_tracking_log = true;
			resetIdentificationState();
		}
	}

	if (_mode == Mode::TRAJECTORY || _mode == Mode::IDENTIFICATION) {
		flushTrackingLogSamples(current_pos, now);
	}

	updateCampaign(current_pos, now, in_offboard);

	if (entered_offboard || _need_reset) {
		resetSmoothers(current_pos, _pos_yaw);
	}

	// when mode is POSITION
	if (_mode == Mode::POSITION) {
		if (!_pos_ref_absolute && (_need_pos_offset || !_pos_offset_valid)) {
			_pos_offset = current_pos;
			_pos_offset_valid = true;
			_need_pos_offset = false;
		}

		const matrix::Vector3f pos_target = _pos_ref_absolute
			? _pos_target
			: (_pos_offset_valid ? (_pos_offset + _pos_target) : _pos_target);

		goto_setpoint_s goto_sp{};
		goto_sp.position[0] = pos_target(0);
		goto_sp.position[1] = pos_target(1);
		goto_sp.position[2] = pos_target(2);
		goto_sp.flag_control_heading = true;
		goto_sp.heading = _pos_yaw;

		setPositionSmootherLimits(goto_sp);
		setHeadingSmootherLimits(goto_sp);

		const float dt = 0.02f;
		matrix::Vector3f pos = current_pos;
		// Use local copies to roll out the horizon, but advance the real smoothers by 1 step.
		auto pos_tmp = _position_smoothing;
		auto yaw_tmp = _heading_smoothing;
		// iterate for HORIZON length
		for (size_t i = 0; i < MAX_HORIZON; i++) {

			PositionSmoothing::PositionSmoothingSetpoints out{};
			pos_tmp.generateSetpoints(
				pos,
				pos_target,
				{},
				dt,
				false,
				out
			);

			pos = out.position;  // feed forward

			yaw_tmp.update(_pos_yaw, dt);

			_buffer[i].px = out.position(0);
			_buffer[i].py = out.position(1);
			_buffer[i].pz = out.position(2);
			_buffer[i].vx = out.velocity(0);
			_buffer[i].vy = out.velocity(1);
			_buffer[i].vz = out.velocity(2);
			_buffer[i].ax = out.acceleration(0);
			_buffer[i].ay = out.acceleration(1);
			_buffer[i].az = out.acceleration(2);
			_buffer[i].yaw = yaw_tmp.getSmoothedHeading();
			_buffer[i].yawspeed = yaw_tmp.getSmoothedHeadingRate();

			if (i == 0) {
				// advance the real smoothers by one step each Run()
				_position_smoothing = pos_tmp;
				_heading_smoothing = yaw_tmp;
			}
		}

		_buffer_len = MAX_HORIZON;
		publishSetpoint(current_pos);
	}

	// when mode is TRAJECTORY
	if (_mode == Mode::TRAJECTORY) {
			if (_fd < 0 && !openTrajectoryFile()) {
				return;
			}

			const bool eof_before = _eof;

			if (loadNextHorizon()) {
				if (_need_traj_offset && _buffer_len > 0) {
					const matrix::Vector3f first_sample{
						_buffer[0].px,
						_buffer[0].py,
						_buffer[0].pz
					};
					const matrix::Vector3f traj_anchor = _traj_anchor_valid ? _traj_anchor : current_pos;
					// Align the first trajectory sample to a fixed anchor when provided.
					_traj_offset = traj_anchor - first_sample;
					_traj_offset_valid = true;
					_need_traj_offset = false;
				}
				publishSetpoint(current_pos);

				if (!eof_before && _eof) {
					_finalize_tracking_log = true;
					flushTrackingLogSamples(current_pos, hrt_absolute_time());
				}
			}
	}

	if (_mode == Mode::IDENTIFICATION) {
		fillIdentificationBuffer(current_pos, now);
		publishSetpoint(current_pos);

		if (_eof) {
			_finalize_tracking_log = true;
			flushTrackingLogSamples(current_pos, now);
			if (_ident_finalize_log) {
				stopIdentificationLog();
			}
		}
	}
}

void TrajectoryReader::setTrajectoryReaderMode(Mode mode)
{
	if (mode == _mode) {
		return;
	}

	_mode = mode;
	clearTrackingLogFailure();

	if (_mode == Mode::TRAJECTORY) {
		stopTrajectoryTrackingLog();
		stopIdentificationLog();
		closeTrajectoryFile();
		_buffer_len = 0;
		_eof = false;
		_finalize_tracking_log = false;
		_traj_offset_valid = false;
		_need_traj_offset = true;
		_start_new_tracking_log = true;
		resetIdentificationState();
	} else if (_mode == Mode::IDENTIFICATION) {
		stopTrajectoryTrackingLog();
		stopIdentificationLog();
		closeTrajectoryFile();
		_buffer_len = 0;
		_finalize_tracking_log = false;
		_start_new_tracking_log = true;
		_traj_offset_valid = false;
		_need_traj_offset = false;
		resetIdentificationState();
	} else {
		_traj_offset_valid = false;
		stopTrajectoryTrackingLog();
		stopIdentificationLog();
		resetIdentificationState();
	}

	if (_mode == Mode::POSITION) {
		_pos_offset_valid = false;
		_need_pos_offset = true;
		_pos_ref_absolute = false;
		_pos_target = {};
	} else {
		_pos_offset_valid = false;
	}
}

void TrajectoryReader::setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	// Horizontal constraints
	float max_horizontal_speed = _mpc_xy_cruise;
	float max_horizontal_accel = _mpc_acc_hor;

	if (goto_setpoint.flag_set_max_horizontal_speed
	    && PX4_ISFINITE(goto_setpoint.max_horizontal_speed)) {
		max_horizontal_speed = math::constrain(goto_setpoint.max_horizontal_speed, 0.f,
						       _mpc_xy_cruise);

		// linearly scale horizontal acceleration limit with horizontal speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (!_position_smoothing.getCurrentVelocityXY().longerThan(max_horizontal_speed)) {
			const float speed_scale = max_horizontal_speed / _mpc_xy_cruise;
			max_horizontal_accel = math::constrain(_mpc_acc_hor * speed_scale, 0.f, _mpc_acc_hor);
		}
	}

	_position_smoothing.setCruiseSpeed(max_horizontal_speed);
	_position_smoothing.setMaxAccelerationXY(max_horizontal_accel);

	// Vertical constraints
	float vehicle_max_vertical_speed = _mpc_z_v_auto_dn;
	float vehicle_max_vertical_accel = _mpc_acc_down_max;

	if (goto_setpoint.position[2] < _position_smoothing.getCurrentPositionZ()) { // goto higher -> more negative
		vehicle_max_vertical_speed = _mpc_z_v_auto_up;
		vehicle_max_vertical_accel = _mpc_acc_up_max;
	}

	float max_vertical_speed = vehicle_max_vertical_speed;
	float max_vertical_accel = vehicle_max_vertical_accel;

	if (goto_setpoint.flag_set_max_vertical_speed && PX4_ISFINITE(goto_setpoint.max_vertical_speed)) {
		max_vertical_speed = math::constrain(goto_setpoint.max_vertical_speed, 0.f, vehicle_max_vertical_speed);

		// linearly scale vertical acceleration limit with vertical speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (fabsf(_position_smoothing.getCurrentVelocityZ()) <= max_vertical_speed) {
			const float speed_scale = max_vertical_speed / vehicle_max_vertical_speed;
			max_vertical_accel = math::constrain(vehicle_max_vertical_accel * speed_scale, 0.f, vehicle_max_vertical_accel);
		}
	}

	_position_smoothing.setMaxVelocityZ(max_vertical_speed);
	_position_smoothing.setMaxAccelerationZ(max_vertical_accel);
}

void TrajectoryReader::setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	float max_heading_rate = _mpc_yawrauto_max;
	float max_heading_accel = _mpc_yawrauto_acc;

	if (goto_setpoint.flag_set_max_heading_rate && PX4_ISFINITE(goto_setpoint.max_heading_rate)) {
		max_heading_rate = math::constrain(goto_setpoint.max_heading_rate, 0.f, _mpc_yawrauto_max);

		// linearly scale heading acceleration limit with heading rate limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		if (fabsf(_heading_smoothing.getSmoothedHeadingRate()) <= max_heading_rate) {
			const float rate_scale = max_heading_rate / _mpc_yawrauto_max;
			max_heading_accel = math::constrain(_mpc_yawrauto_acc * rate_scale, 0.f, _mpc_yawrauto_acc);
		}
	}

	_heading_smoothing.setMaxHeadingRate(max_heading_rate);
	_heading_smoothing.setMaxHeadingAccel(max_heading_accel);
}

void TrajectoryReader::setPositionModeRef(const matrix::Vector3f &pos, float yaw, bool absolute)
{
	_pos_target = pos;
	_pos_yaw = yaw;
	_pos_ref_absolute = absolute;
	_need_pos_offset = !absolute;

	if (absolute) {
		_pos_offset_valid = false;
	}

	_need_reset = true;
}

void TrajectoryReader::setTrajectoryAnchor(const matrix::Vector3f &pos)
{
	_traj_anchor = pos;
	_traj_anchor_valid = true;
	_traj_offset_valid = false;
	_need_traj_offset = true;
}

void TrajectoryReader::resetSmoothers(const matrix::Vector3f &pos, float yaw)
{
	_position_smoothing.reset({}, {}, pos);
	_heading_smoothing.reset(yaw, 0.f);
	_need_reset = false;
}

bool TrajectoryReader::setTrajectoryId(uint8_t traj_id)
{
	if (traj_id == _traj_id) {
		return true;
	}

	const uint8_t previous_id = _traj_id;
	_traj_id = traj_id;

	stopTrajectoryTrackingLog();
	closeTrajectoryFile();
	_buffer_len = 0;
	_eof = false;
	_finalize_tracking_log = false;

	if (_mode == Mode::TRAJECTORY) {
		_start_new_tracking_log = true;
	}

	if (_mode == Mode::TRAJECTORY) {
		if (!openTrajectoryFile()) {
			_traj_id = previous_id;
			openTrajectoryFile();
			return false;
		}
	}

	return true;
}


int TrajectoryReader::task_spawn(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (ModuleBase::is_running(desc)) {
        PX4_WARN("already running");
        return PX4_OK;
    }

    TrajectoryReader *instance = new TrajectoryReader();

    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    if (!instance->init()) {
        PX4_ERR("init failed");
        delete instance;
        desc.object.store(nullptr);
        desc.task_id = -1;
        return PX4_ERROR;
    }

    desc.object.store(instance);
    desc.task_id = task_id_is_work_queue;

    return PX4_OK;
}

int TrajectoryReader::custom_command(int argc, char *argv[])
{
    if (!ModuleBase::is_running(desc)) {
        PX4_WARN("module not running");
        return PX4_ERROR;
    }

    TrajectoryReader *instance = trajectoryReaderInstance();

    if (!instance) {
        return PX4_ERROR;
    }

	if (!strcmp(argv[0], "reload")) {
		PX4_INFO("Reloading trajectory");
		instance->clearTrackingLogFailure();
		instance->stopTrajectoryTrackingLog();
		instance->_finalize_tracking_log = false;
		instance->_start_new_tracking_log = (instance->_mode == Mode::TRAJECTORY);
		instance->closeTrajectoryFile();
		instance->_buffer_len = 0;
		instance->_eof = false;
        return PX4_OK;
    }

	if (!strcmp(argv[0], "rewind")) {
		PX4_INFO("Rewinding trajectory");
		instance->clearTrackingLogFailure();
		instance->stopTrajectoryTrackingLog();
		instance->_finalize_tracking_log = false;
		instance->_start_new_tracking_log = (instance->_mode == Mode::TRAJECTORY);
		instance->closeTrajectoryFile();
		instance->openTrajectoryFile();
		instance->_buffer_len = 0;
        return PX4_OK;
    }

    if (!strcmp(argv[0], "ref")) {
        if (argc < 5) {
            PX4_ERR("usage: trajectory_reader ref <x> <y> <z> <yaw>");
            return PX4_ERROR;
        }

        float x = strtof(argv[1], nullptr);
        float y = strtof(argv[2], nullptr);
        float z = strtof(argv[3], nullptr);
        const matrix::Vector3f pos_target(x, y, z);
        float yaw = strtof(argv[4], nullptr);

        instance->setPositionModeRef(pos_target, yaw, false);
        
        PX4_INFO("Position reference set to [%.2f %.2f %.2f] yaw %.2f", (double)x, (double)y, (double)z, (double)yaw);
        return PX4_OK;
    }

    if (!strcmp(argv[0], "abs_ref")) {
        if (argc < 5) {
            PX4_ERR("usage: trajectory_reader abs_ref <x> <y> <z> <yaw>");
            return PX4_ERROR;
        }

        const float x = strtof(argv[1], nullptr);
        const float y = strtof(argv[2], nullptr);
        const float z = strtof(argv[3], nullptr);
        const matrix::Vector3f pos_target(x, y, z);
        const float yaw = strtof(argv[4], nullptr);

        instance->setPositionModeRef(pos_target, yaw, true);

        PX4_INFO("Absolute position reference set to [%.2f %.2f %.2f] yaw %.2f",
                 (double)x, (double)y, (double)z, (double)yaw);
        return PX4_OK;
    }

	if (!strcmp(argv[0], "set_traj_anchor")) {
		if (argc < 4) {
			PX4_ERR("usage: trajectory_reader set_traj_anchor <x> <y> <z>");
			return PX4_ERROR;
		}

		const float x = strtof(argv[1], nullptr);
		const float y = strtof(argv[2], nullptr);
		const float z = strtof(argv[3], nullptr);

		instance->setTrajectoryAnchor(matrix::Vector3f(x, y, z));
		const float anchor_values[3] = {x, y, z};
		param_t anchor_params[3] = {
			param_find("TRJ_ANCHOR_X"),
			param_find("TRJ_ANCHOR_Y"),
			param_find("TRJ_ANCHOR_Z"),
		};
		for (int i = 0; i < 3; ++i) {
			if (anchor_params[i] != PARAM_INVALID) {
				param_set(anchor_params[i], &anchor_values[i]);
			}
		}
		PX4_INFO("Trajectory anchor set to [%.2f %.2f %.2f]", (double)x, (double)y, (double)z);
		return PX4_OK;
	}

    if (!strcmp(argv[0], "set_mode")) {
        if (argc < 2) {
            PX4_ERR("usage: trajectory_reader set_mode <position|trajectory|identification>");
            return PX4_ERROR;
        }

        Mode mode;
        if (!strcmp(argv[1], "position")) {
            mode = Mode::POSITION;
        } else if (!strcmp(argv[1], "trajectory")) {
            mode = Mode::TRAJECTORY;
        } else if (!strcmp(argv[1], "identification")) {
            mode = Mode::IDENTIFICATION;
        } else {
            PX4_ERR("Invalid mode: %s", argv[1]);
            return PX4_ERROR;
        }

        instance->setTrajectoryReaderMode(mode);
		const int32_t mode_value = static_cast<int32_t>(mode);
		param_t mode_param = param_find("TRJ_MODE_CMD");
		if (mode_param != PARAM_INVALID) {
			param_set(mode_param, &mode_value);
		}
        return PX4_OK;
    }

    if (!strcmp(argv[0], "set_ident_profile")) {
        if (argc < 2) {
            PX4_ERR("usage: trajectory_reader set_ident_profile <hover_thrust|roll_sweep|pitch_sweep|yaw_sweep|drag_x|drag_y|drag_z|mass_vertical|motor_step>");
            return PX4_ERROR;
        }

        IdentificationProfile profile = IdentificationProfile::HOVER_THRUST;
        if (!strcmp(argv[1], "hover_thrust")) {
            profile = IdentificationProfile::HOVER_THRUST;
        } else if (!strcmp(argv[1], "roll_sweep")) {
            profile = IdentificationProfile::ROLL_SWEEP;
        } else if (!strcmp(argv[1], "pitch_sweep")) {
            profile = IdentificationProfile::PITCH_SWEEP;
        } else if (!strcmp(argv[1], "yaw_sweep")) {
            profile = IdentificationProfile::YAW_SWEEP;
        } else if (!strcmp(argv[1], "drag_x")) {
            profile = IdentificationProfile::DRAG_X;
        } else if (!strcmp(argv[1], "drag_y")) {
            profile = IdentificationProfile::DRAG_Y;
        } else if (!strcmp(argv[1], "drag_z")) {
            profile = IdentificationProfile::DRAG_Z;
        } else if (!strcmp(argv[1], "mass_vertical")) {
            profile = IdentificationProfile::MASS_VERTICAL;
        } else if (!strcmp(argv[1], "motor_step")) {
            profile = IdentificationProfile::MOTOR_STEP;
        } else {
            PX4_ERR("Invalid identification profile: %s", argv[1]);
            return PX4_ERROR;
        }

        instance->_ident_profile = profile;
        instance->clearTrackingLogFailure();
        instance->stopTrajectoryTrackingLog();
        instance->stopIdentificationLog();
        instance->resetIdentificationState();
        instance->_start_new_tracking_log = (instance->_mode == Mode::IDENTIFICATION);
        const int32_t profile_value = static_cast<int32_t>(profile);
        param_t ident_param = param_find("TRJ_IDENT_PROF");
        if (ident_param != PARAM_INVALID) {
            param_set(ident_param, &profile_value);
        }
        PX4_INFO("Identification profile set to %s", instance->identProfileToString(profile));
        PX4_INFO("  Purpose: %s", instance->identProfilePurpose(profile));
        PX4_INFO("  Estimated duration: %.1f s", (double)instance->identificationDurationS(profile));
        return PX4_OK;
    }

	if (!strcmp(argv[0], "set_campaign")) {
		if (argc < 2) {
			PX4_ERR("usage: trajectory_reader set_campaign <none|identification_only|trajectory_only|full_stack>");
			return PX4_ERROR;
		}

		CampaignType campaign_type = CampaignType::NONE;
		if (!strcmp(argv[1], "none")) {
			campaign_type = CampaignType::NONE;
		} else if (!strcmp(argv[1], "identification_only")) {
			campaign_type = CampaignType::IDENTIFICATION_ONLY;
		} else if (!strcmp(argv[1], "trajectory_only")) {
			campaign_type = CampaignType::TRAJECTORY_ONLY;
		} else if (!strcmp(argv[1], "full_stack")) {
			campaign_type = CampaignType::FULL_STACK;
		} else {
			PX4_ERR("Invalid campaign: %s", argv[1]);
			return PX4_ERROR;
		}

		instance->_campaign_type = campaign_type;
		const int32_t campaign_value = static_cast<int32_t>(campaign_type);
		param_t campaign_param = param_find("TRJ_CAMPAIGN");
		if (campaign_param != PARAM_INVALID) {
			param_set(campaign_param, &campaign_value);
		}

		if (campaign_type == CampaignType::NONE) {
			instance->stopCampaign(true);
		}

		PX4_INFO("Campaign set to %s", instance->campaignTypeToString(campaign_type));
		return PX4_OK;
	}

	if (!strcmp(argv[0], "start_campaign")) {
		if (!instance->requestCampaignStart()) {
			return PX4_ERROR;
		}

		const int32_t start_value = 1;
		param_t campaign_cmd = param_find("TRJ_CAMPAIGN_CMD");
		if (campaign_cmd != PARAM_INVALID) {
			param_set(campaign_cmd, &start_value);
		}
		return PX4_OK;
	}

	if (!strcmp(argv[0], "stop_campaign")) {
		instance->stopCampaign(true);
		const int32_t stop_value = 2;
		param_t campaign_cmd = param_find("TRJ_CAMPAIGN_CMD");
		if (campaign_cmd != PARAM_INVALID) {
			param_set(campaign_cmd, &stop_value);
		}
		return PX4_OK;
	}

    if (!strcmp(argv[0], "set_traj_id")) {
        if (argc < 2) {
            PX4_ERR("usage: trajectory_reader set_traj_id <trajectory_id_number>");
            return PX4_ERROR;
        }

        char *end = nullptr;
        unsigned long id = strtoul(argv[1], &end, 10);

        if (end == argv[1] || *end != '\0' || id > UINT8_MAX) {
            PX4_ERR("Invalid trajectory id: %s", argv[1]);
            return PX4_ERROR;
        }

        if (!instance->setTrajectoryId(static_cast<uint8_t>(id))) {
            PX4_ERR("Failed to set trajectory id %lu", id);
            return PX4_ERROR;
        }

		const int32_t traj_param_value = static_cast<int32_t>(id);
		param_t traj_param = param_find("TRJ_ACTIVE_ID");
		if (traj_param != PARAM_INVALID) {
			param_set(traj_param, &traj_param_value);
		}

        PX4_INFO("Trajectory id set to %lu", id);
        return PX4_OK;
    }

    return print_usage("Unknown command");
}

int TrajectoryReader::print_status()
{
    const char *mode_name = "POSITION";
    if (_mode == Mode::TRAJECTORY) {
        mode_name = "TRAJECTORY";
    } else if (_mode == Mode::IDENTIFICATION) {
        mode_name = "IDENTIFICATION";
    }
    PX4_INFO("Trajectory Reader status:");
    PX4_INFO("  Mode: %s", mode_name);
    PX4_INFO("  Trajectory ID: %u", _traj_id);
    PX4_INFO("  File open: %s", (_fd >= 0) ? "yes" : "no");
    PX4_INFO("  Samples read: %u / %u",
             static_cast<unsigned>(_cursor),
             static_cast<unsigned>(_num_samples));
	PX4_INFO("  EOF: %s", _eof ? "yes" : "no");
	PX4_INFO("  Horizon size: %u",
		 static_cast<unsigned>(_buffer_len));
	PX4_INFO("  Tracking log open: %s", (_tracking_log_fd >= 0) ? "yes" : "no");
	PX4_INFO("  Pending tracking samples: %u",
		 static_cast<unsigned>(_pending_tracking_size));
	PX4_INFO("  Tracking log fail count: %u",
		 static_cast<unsigned>(_tracking_log_fail_count));
	PX4_INFO("  Tracking log latched: %s", _tracking_log_failed_latched ? "yes" : "no");
	PX4_INFO("  Identification profile: %s", identProfileToString(_ident_profile));
	PX4_INFO("  Identification purpose: %s", identProfilePurpose(_ident_profile));
	PX4_INFO("  Identification duration: %.2f s", (double)identificationDurationS(_ident_profile));
	PX4_INFO("  Identification log open: %s", (_ident_log_fd >= 0) ? "yes" : "no");
	PX4_INFO("  Campaign: %s", campaignTypeToString(_campaign_type));
	PX4_INFO("  Campaign active: %s", _campaign_active ? "yes" : "no");
	PX4_INFO("  Campaign stage: %u", static_cast<unsigned>(_campaign_stage));
	PX4_INFO("  Campaign item index: %u", static_cast<unsigned>(_campaign_item_index));

    if (_mode == Mode::POSITION) {
        PX4_INFO("  Position target: [%.2f, %.2f, %.2f]", (double)_pos_target(0), (double)_pos_target(1), (double)_pos_target(2));
        PX4_INFO("  Position yaw: %.2f rad", (double)_pos_yaw);
        PX4_INFO("  Position reference frame: %s", _pos_ref_absolute ? "ABSOLUTE" : "RELATIVE");
    }

	PX4_INFO("  Trajectory anchor valid: %s", _traj_anchor_valid ? "yes" : "no");

	if (_traj_anchor_valid) {
		PX4_INFO("  Trajectory anchor: [%.2f, %.2f, %.2f]",
			 (double)_traj_anchor(0), (double)_traj_anchor(1), (double)_traj_anchor(2));
	}

    return PX4_OK;
}

int TrajectoryReader::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Publishes setpoints for two workflows used by this repository:
- baseline PX4 position-control references
- built-in system-identification motions with synchronized logging

The module can read prerecorded `.traj` files or generate identification
profiles such as hover, inertia sweeps, drag sweeps, and motor-step tests.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("trajectory_reader", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");

    PRINT_MODULE_USAGE_COMMAND_DESCR("reload",
        "Reload trajectory file and restart from beginning");

    PRINT_MODULE_USAGE_COMMAND_DESCR("rewind",
        "Alias for reload");

    PRINT_MODULE_USAGE_COMMAND_DESCR("ref",
        "Set position reference");
    PRINT_MODULE_USAGE_ARG("<x> <y> <z> <yaw>", "Position coordinates (m), yaw (rad)", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("abs_ref",
        "Set absolute position reference");
    PRINT_MODULE_USAGE_ARG("<x> <y> <z> <yaw>", "Absolute coordinates (m), yaw (rad)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_traj_anchor",
		"Set fixed trajectory anchor in local coordinates");
	PRINT_MODULE_USAGE_ARG("<x> <y> <z>", "Anchor coordinates (m)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_mode",
		"Set operating mode");
	PRINT_MODULE_USAGE_ARG("<position|trajectory|identification>", "Mode to set", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_traj_id",
        "Set trajectory id to load");
    PRINT_MODULE_USAGE_ARG("<trajectory_id_number>", "Trajectory id (0-255)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_ident_profile",
		"Set identification motion profile");
	PRINT_MODULE_USAGE_ARG("<hover_thrust|roll_sweep|pitch_sweep|yaw_sweep|drag_x|drag_y|drag_z|mass_vertical|motor_step>",
		"Identification profile", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_campaign",
		"Select the built-in campaign");
	PRINT_MODULE_USAGE_ARG("<none|identification_only|trajectory_only|full_stack>",
		"Campaign to arm", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("start_campaign",
		"Start the selected campaign once OFFBOARD is active");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop_campaign",
		"Abort the active campaign and hold position");

    PRINT_MODULE_USAGE_PARAM_INT(
        'i',
        0,
        0,
        255,
        "Trajectory ID to load from storage",
        true
    );

    return PX4_OK;
}

extern "C" __EXPORT int trajectory_reader_main(int argc, char *argv[])
{
    return ModuleBase::main(TrajectoryReader::desc, argc, argv);
}
