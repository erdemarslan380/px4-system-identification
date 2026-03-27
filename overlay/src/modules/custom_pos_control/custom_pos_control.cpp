#include "custom_pos_control.hpp"

#include <cstring>

#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

namespace
{
CustomPosControl *customPosControlInstance()
{
	return ModuleBase::get_instance<CustomPosControl>(CustomPosControl::desc);
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
}

ModuleBase::Descriptor CustomPosControl::desc{task_spawn, custom_command, print_usage};

CustomPosControl::CustomPosControl() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	ModuleParams(nullptr)
{
	_ref = {};
	_ref.valid = false;
	_ref.horizon_length = 0;
}

CustomPosControl::~CustomPosControl()
{
	ScheduleClear();
}

bool CustomPosControl::init()
{
	parametersUpdate();
	switchController(static_cast<ControllerType>(_controller_type));
	ScheduleOnInterval(10_ms);
	return true;
}

void CustomPosControl::parametersUpdate()
{
	ModuleParams::updateParams();
	_enabled = _param_cst_pos_en.get();
	_controller_type = static_cast<uint8_t>(_param_cst_pos_typ.get());
	_rc_select_enabled = _param_cst_rc_sel_en.get();
	_rc_controller_channel = _param_cst_rc_ctrl_ch.get();
}

bool CustomPosControl::isActiveForwardingController() const
{
	return _active_type == ControllerType::PX4_DEFAULT || _active_type == ControllerType::SYSID;
}

void CustomPosControl::updateRcControllerSelection()
{
	if (_rc_select_enabled == 0 || _rc_controller_channel < 1 || _rc_controller_channel > 6) {
		_rc_selected_slot = -1;
		return;
	}

	manual_control_setpoint_s manual_control{};
	if (!_manual_control_sub.update(&manual_control) || !manual_control.valid) {
		return;
	}

	static constexpr ControllerType selectable_controllers[] = {
		ControllerType::PX4_DEFAULT,
		ControllerType::SYSID,
	};

	const float aux_value = readAuxChannel(manual_control, _rc_controller_channel);
	const int slot = quantizeAuxSelection(aux_value, static_cast<int>(sizeof(selectable_controllers) / sizeof(selectable_controllers[0])));
	if (slot < 0 || slot == _rc_selected_slot) {
		return;
	}

	_rc_selected_slot = slot;
	const ControllerType new_type = selectable_controllers[slot];
	const int32_t param_value = static_cast<int32_t>(new_type);
	param_t param = param_find("CST_POS_CTRL_TYP");
	if (param != PARAM_INVALID) {
		param_set(param, &param_value);
	}

	_controller_type = static_cast<uint8_t>(new_type);
	switchController(new_type);
}

void CustomPosControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_param_update_sub.updated()) {
		parametersUpdate();
		switchController(static_cast<ControllerType>(_controller_type));
	}

	updateRcControllerSelection();

	vehicle_status_s status{};
	_status_sub.copy(&status);
	const bool controller_active = isActiveForwardingController();

	auto should_log_offboard_pause = [&](uint8_t reason_id) -> bool {
		static hrt_abstime last_log_time = 0;
		static uint8_t last_reason = 0;
		const hrt_abstime now = hrt_absolute_time();

		if (reason_id != last_reason || (now - last_log_time) > 1_s) {
			last_reason = reason_id;
			last_log_time = now;
			return true;
		}

		return false;
	};

	if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED || _enabled == 0 || !controller_active) {
		if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
				if (should_log_offboard_pause(1)) {
					PX4_WARN("Offboard output paused: vehicle not armed");
				}
			} else if (_enabled == 0) {
				if (should_log_offboard_pause(2)) {
					PX4_WARN("Offboard output paused: CST_POS_CTRL_EN=0");
				}
			} else {
				if (should_log_offboard_pause(3)) {
					PX4_WARN("Offboard output paused: controller not set to PX4_DEFAULT or SYSID");
				}
			}
		}
		return;
	}

	vehicle_local_position_s local_pos{};
	if (_local_pos_sub.update(&local_pos)) {
		_local_pos_cached = local_pos;
		_last_local_pos_time = local_pos.timestamp;
	}

	if (_last_local_pos_time == 0) {
		if (should_log_offboard_pause(4)) {
			PX4_WARN("Offboard output paused: no local position received yet");
		}
		return;
	}

	if (hrt_elapsed_time(&_last_local_pos_time) > LOCAL_POS_TIMEOUT) {
		if (should_log_offboard_pause(5)) {
			PX4_WARN("Offboard output paused: local position stale");
		}
		return;
	}

	const vehicle_local_position_s local_pos_cached = _local_pos_cached;
	const hrt_abstime now = hrt_absolute_time();

	if (_multi_ref_sub.update(&_traj_sp_tmp)) {
		_ref = _traj_sp_tmp;
		if (_ref.horizon_length > multi_trajectory_setpoint_s::MAX_HORIZON) {
			_ref.horizon_length = multi_trajectory_setpoint_s::MAX_HORIZON;
		}
		_ref.timestamp = now;
		_last_ref_time = now;
	} else if (_last_ref_time == 0 || hrt_elapsed_time(&_last_ref_time) > REF_TIMEOUT) {
		_ref = {};
		_ref.position_x[0] = local_pos_cached.x;
		_ref.position_y[0] = local_pos_cached.y;
		_ref.position_z[0] = local_pos_cached.z;
		_ref.velocity_x[0] = 0.0f;
		_ref.velocity_y[0] = 0.0f;
		_ref.velocity_z[0] = 0.0f;
		_ref.accel_x[0] = 0.0f;
		_ref.accel_y[0] = 0.0f;
		_ref.accel_z[0] = 0.0f;
		_ref.yaw[0] = local_pos_cached.heading_good_for_control ? local_pos_cached.heading : 0.0f;
		_ref.yawspeed[0] = 0.0f;
		_ref.valid = true;
		_ref.horizon_length = 1;
		_ref.timestamp = now;
	}

	adjustSetpointForEKFResets(local_pos_cached, _ref);
	publishOffboardControlMode(now);
	publishForwardedSetpoint(now);
}

void CustomPosControl::publishOffboardControlMode(const hrt_abstime now)
{
	offboard_control_mode_s ocm{};
	ocm.timestamp = now;
	ocm.position = true;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	_offboard_control_mode_pub.publish(ocm);
}

void CustomPosControl::publishForwardedSetpoint(const hrt_abstime now)
{
	trajectory_setpoint_s trajectory_sp{};
	trajectory_sp.timestamp = now;

	if (_ref.valid && _ref.horizon_length >= 1) {
		trajectory_sp.position[0] = _ref.position_x[0];
		trajectory_sp.position[1] = _ref.position_y[0];
		trajectory_sp.position[2] = _ref.position_z[0];
		trajectory_sp.velocity[0] = _ref.velocity_x[0];
		trajectory_sp.velocity[1] = _ref.velocity_y[0];
		trajectory_sp.velocity[2] = _ref.velocity_z[0];
		trajectory_sp.acceleration[0] = _ref.accel_x[0];
		trajectory_sp.acceleration[1] = _ref.accel_y[0];
		trajectory_sp.acceleration[2] = _ref.accel_z[0];
		trajectory_sp.yaw = _ref.yaw[0];
		trajectory_sp.yawspeed = _ref.yawspeed[0];
	} else {
		trajectory_sp.position[0] = NAN;
		trajectory_sp.position[1] = NAN;
		trajectory_sp.position[2] = NAN;
		trajectory_sp.velocity[0] = NAN;
		trajectory_sp.velocity[1] = NAN;
		trajectory_sp.velocity[2] = NAN;
		trajectory_sp.acceleration[0] = NAN;
		trajectory_sp.acceleration[1] = NAN;
		trajectory_sp.acceleration[2] = NAN;
		trajectory_sp.yaw = NAN;
		trajectory_sp.yawspeed = NAN;
	}

	trajectory_sp.jerk[0] = NAN;
	trajectory_sp.jerk[1] = NAN;
	trajectory_sp.jerk[2] = NAN;
	_trajectory_sp_pub.publish(trajectory_sp);
}

void CustomPosControl::adjustSetpointForEKFResets(const vehicle_local_position_s &local_pos,
					       multi_trajectory_setpoint_s &setpoint)
{
	const uint8_t horizon_length = math::min(setpoint.horizon_length, multi_trajectory_setpoint_s::MAX_HORIZON);

	if ((setpoint.timestamp != 0) && (setpoint.timestamp < local_pos.timestamp)) {
		for (uint8_t i = 0; i < horizon_length; ++i) {
			if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
				setpoint.velocity_x[i] += local_pos.delta_vxy[0];
				setpoint.velocity_y[i] += local_pos.delta_vxy[1];
			}

			if (local_pos.vz_reset_counter != _vz_reset_counter) {
				setpoint.velocity_z[i] += local_pos.delta_vz;
			}

			if (local_pos.xy_reset_counter != _xy_reset_counter) {
				setpoint.position_x[i] += local_pos.delta_xy[0];
				setpoint.position_y[i] += local_pos.delta_xy[1];
			}

			if (local_pos.z_reset_counter != _z_reset_counter) {
				setpoint.position_z[i] += local_pos.delta_z;
			}

			if (local_pos.heading_reset_counter != _heading_reset_counter) {
				setpoint.yaw[i] = matrix::wrap_pi(setpoint.yaw[i] + local_pos.delta_heading);
			}
		}
	}

	_vxy_reset_counter = local_pos.vxy_reset_counter;
	_vz_reset_counter = local_pos.vz_reset_counter;
	_xy_reset_counter = local_pos.xy_reset_counter;
	_z_reset_counter = local_pos.z_reset_counter;
	_heading_reset_counter = local_pos.heading_reset_counter;
}

void CustomPosControl::switchController(ControllerType new_type)
{
	if (new_type == _active_type) {
		return;
	}

	switch (new_type) {
	case ControllerType::NO_CTRL:
		PX4_WARN("Custom position forwarding disabled");
		break;
	case ControllerType::PX4_DEFAULT:
		PX4_INFO("PX4 baseline position forwarding activated");
		break;
	case ControllerType::SYSID:
		PX4_INFO("System identification forwarding activated");
		break;
	}

	_active_type = new_type;
}

int CustomPosControl::task_spawn(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	if (ModuleBase::is_running(desc)) {
		PX4_WARN("already running");
		return 0;
	}

	CustomPosControl *instance = new CustomPosControl();
	if (!instance) {
		PX4_ERR("allocation failed");
		return -1;
	}

	if (!instance->init()) {
		PX4_ERR("init failed");
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return -1;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	PX4_INFO("custom_pos_control started (work queue)");
	return 0;
}

int CustomPosControl::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	CustomPosControl *instance = customPosControlInstance();
	if (!instance) {
		PX4_WARN("module not running");
		return 0;
	}

	if (!strcmp(argv[0], "enable")) {
		int32_t val = 1;
		param_set(param_find("CST_POS_CTRL_EN"), &val);
		PX4_INFO("custom_pos_control enabled (parameter set)");
		return 0;
	}

	if (!strcmp(argv[0], "disable")) {
		int32_t val = 0;
		param_set(param_find("CST_POS_CTRL_EN"), &val);
		PX4_INFO("custom_pos_control disabled (parameter set)");
		return 0;
	}

	if (!strcmp(argv[0], "set")) {
		if (argc < 2) {
			return print_usage("set <none|px4_default|sysid>");
		}

		int32_t val = -1;
		if (!strcmp(argv[1], "none")) val = 0;
		if (!strcmp(argv[1], "px4_default")) val = 4;
		if (!strcmp(argv[1], "sysid")) val = 6;

		if (val < 0) {
			PX4_ERR("unknown controller type");
			return 0;
		}

		param_set(param_find("CST_POS_CTRL_TYP"), &val);
		PX4_INFO("controller type set to %d", static_cast<int>(val));
		return 0;
	}

	return print_usage("unknown command");
}

int CustomPosControl::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Enabled: %d", static_cast<int>(_enabled));
	PX4_INFO("Controller type: %d", static_cast<int>(_active_type));
	PX4_INFO("0: No Controller");
	PX4_INFO("4: PX4_DEFAULT");
	PX4_INFO("6: SYSID");
	return 0;
}

int CustomPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal offboard forwarding module used by the PX4 System Identification workspace.
It only supports two active modes:
- PX4_DEFAULT: forwards the reference to the standard PX4 multicopter controller
- SYSID: forwards the reference while the trajectory_reader generates identification motions
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("enable");
	PRINT_MODULE_USAGE_COMMAND("disable");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Select forwarding mode");
	PRINT_MODULE_USAGE_ARG("<none|px4_default|sysid>", "Controller type", false);
	return 0;
}

extern "C" __EXPORT int custom_pos_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomPosControl::desc, argc, argv);
}
