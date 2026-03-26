// custom_pos_control.cpp
// source file for CustomPosControl

#include "custom_pos_control.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

#include <parameters/param.h>

namespace
{
bool finiteQuaternion(const float q[4])
{
    return PX4_ISFINITE(q[0]) && PX4_ISFINITE(q[1]) && PX4_ISFINITE(q[2]) && PX4_ISFINITE(q[3]);
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

CustomPosControl::CustomPosControl() :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::custom_pos_control),
    ModuleParams(nullptr) {

    PX4_INFO("CustomPosControl constructed");

    _ref = {};
    _ref.valid = false;
    _ref.horizon_length = 0;
}

CustomPosControl::~CustomPosControl() {
    PX4_INFO("CustomPosControl destructed");

    ScheduleClear();
    delete _active_controller;
}

bool CustomPosControl::init() {
    parametersUpdate();

    switchController(static_cast<ControllerType>(_controller_type));
    ScheduleOnInterval(4000_us);
    return true;
}

void CustomPosControl::parametersUpdate() {
    ModuleParams::updateParams();
    _enabled = _param_cst_pos_en.get();
    _controller_type = (uint8_t)_param_cst_pos_typ.get();
    _rc_select_enabled = _param_cst_rc_sel_en.get();
    _rc_controller_channel = _param_cst_rc_ctrl_ch.get();

    const float sample_freq_hz = 1.f / _sample_interval_s.mean();

    // velocity notch filter
    if ((_param_mpc_vel_nf_frq.get() > 0.f) && (_param_mpc_vel_nf_bw.get() > 0.f)) {
        _vel_xy_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());
        _vel_z_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());

    } else {
        _vel_xy_notch_filter.disable();
        _vel_z_notch_filter.disable();
    }

    // velocity xy/z low pass filter
    if (_param_mpc_vel_lp.get() > 0.f) {
        _vel_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());
        _vel_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());

    } else {
        // disable filtering
        _vel_xy_lp_filter.setAlpha(1.f);
        _vel_z_lp_filter.setAlpha(1.f);
    }

    // velocity derivative xy/z low pass filter
    if (_param_mpc_veld_lp.get() > 0.f) {
        _vel_deriv_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());
        _vel_deriv_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());

    } else {
        // disable filtering
        _vel_deriv_xy_lp_filter.setAlpha(1.f);
        _vel_deriv_z_lp_filter.setAlpha(1.f);
    }
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
        ControllerType::DFBC,
        ControllerType::MPC,
        ControllerType::CMPC,
        ControllerType::INDI,
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

void CustomPosControl::Run() {
    // exit guard
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // if parameters are updated
    // we updated the controller parameters
    if (_param_update_sub.updated()) {
        parametersUpdate();
        switchController(static_cast<ControllerType>(_controller_type));

        if (_active_controller) {
            _active_controller->parametersUpdate();
        }

        _attitude_mpc_controller.parametersUpdate();
    }

    updateRcControllerSelection();

    // only run when armed and enabled and controller is active
    vehicle_status_s status{};
    _status_sub.copy(&status);
    const bool using_px4_default = (_active_type == ControllerType::PX4_DEFAULT || _active_type == ControllerType::SYSID);

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

    if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
        _enabled == 0 ||
        (!using_px4_default && !_active_controller) ||
        _active_type == ControllerType::NO_CTRL) {
        if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
            if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
                if (should_log_offboard_pause(1)) {
                    PX4_WARN("Offboard output paused: vehicle not armed");
                }
            } else if (_enabled == 0) {
                if (should_log_offboard_pause(2)) {
                    PX4_WARN("Offboard output paused: CST_POS_CTRL_EN=0");
                }
            } else if (_active_type == ControllerType::NO_CTRL) {
                if (should_log_offboard_pause(3)) {
                    PX4_WARN("Offboard output paused: controller type NO_CTRL");
                }
            } else if (!using_px4_default && !_active_controller) {
                if (should_log_offboard_pause(4)) {
                    PX4_WARN("Offboard output paused: controller not initialized");
                }
            }
        }
        return;
    }

    publishOffboardControlMode(hrt_absolute_time());

    hover_thrust_estimate_s hover{};
    // estimating hover thrust
    if (_hover_thrust_sub.update(&hover) && hover.valid) {
        if (_active_controller) {
            _active_controller->setHoverThrust(hover.hover_thrust);
        }
    }

    if (!using_px4_default) {
        // update attitude and body rates for the custom attitude MPC stage
        vehicle_attitude_s attitude{};
        if (_att_sub.update(&attitude)) {
            _att_cached = attitude;
            _last_att_time = attitude.timestamp;
        }

        vehicle_angular_velocity_s angular_velocity{};
        if (_ang_vel_sub.update(&angular_velocity)) {
            _ang_vel_cached = angular_velocity;
            _last_ang_vel_time = angular_velocity.timestamp;
        }

        if (_last_att_time == 0) {
            if (should_log_offboard_pause(5)) {
                PX4_WARN("Offboard output paused: no attitude received yet");
            }
            return;
        }

        if (hrt_elapsed_time(&_last_att_time) > ATTITUDE_TIMEOUT) {
            if (should_log_offboard_pause(6)) {
                PX4_WARN("Offboard output paused: attitude stale (%llu us)",
                         static_cast<unsigned long long>(hrt_elapsed_time(&_last_att_time)));
            }
            return;
        }
    }

    // do not run if _local_pos_sub is not updating local position
    vehicle_local_position_s local_pos{};
    if (_local_pos_sub.update(&local_pos)) {
        _local_pos_cached = local_pos;
        _last_local_pos_time = local_pos.timestamp;

        const float dt =
            math::constrain(((local_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
        _time_stamp_last_loop = local_pos.timestamp_sample;
        _dt_s = dt;
        _sample_interval_s.update(dt);
    }

    // if no local position received, do not run
    if (_last_local_pos_time == 0) {
        if (should_log_offboard_pause(7)) {
            PX4_WARN("Offboard output paused: no local position received yet");
        }
        return;
    }

    if (hrt_elapsed_time(&_last_local_pos_time) > LOCAL_POS_TIMEOUT) {
        if (should_log_offboard_pause(8)) {
            PX4_WARN("Offboard output paused: local position stale (%llu us)",
                     static_cast<unsigned long long>(hrt_elapsed_time(&_last_local_pos_time)));
        }
        return;
    }

    const vehicle_local_position_s local_pos_cached = _local_pos_cached;
    vehicle_local_position_s local_pos_filtered = local_pos_cached;

    const matrix::Vector2f velocity_xy(local_pos_cached.vx, local_pos_cached.vy);

    if (local_pos_cached.v_xy_valid && velocity_xy.isAllFinite()) {
        const matrix::Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();

        // vel xy notch filter, then low pass filter
        const matrix::Vector2f vel_xy_filtered = _vel_xy_lp_filter.update(_vel_xy_notch_filter.apply(velocity_xy));
        local_pos_filtered.vx = vel_xy_filtered(0);
        local_pos_filtered.vy = vel_xy_filtered(1);

        // vel xy derivative low pass filter
        const matrix::Vector2f vel_xy_deriv = _vel_deriv_xy_lp_filter.update(
            (_vel_xy_lp_filter.getState() - vel_xy_prev) / _dt_s);
        local_pos_filtered.ax = vel_xy_deriv(0);
        local_pos_filtered.ay = vel_xy_deriv(1);

    } else {
        local_pos_filtered.vx = NAN;
        local_pos_filtered.vy = NAN;
        local_pos_filtered.ax = NAN;
        local_pos_filtered.ay = NAN;

        // reset filters to prevent acceleration spikes when regaining velocity
        _vel_xy_lp_filter.reset({});
        _vel_xy_notch_filter.reset();
        _vel_deriv_xy_lp_filter.reset({});
    }

    if (PX4_ISFINITE(local_pos_cached.vz) && local_pos_cached.v_z_valid) {
        const float vel_z_prev = _vel_z_lp_filter.getState();

        // vel z notch filter, then low pass filter
        const float vel_z_filtered = _vel_z_lp_filter.update(_vel_z_notch_filter.apply(local_pos_cached.vz));
        local_pos_filtered.vz = vel_z_filtered;

        // vel z derivative low pass filter
        const float vel_z_deriv = _vel_deriv_z_lp_filter.update(
            (_vel_z_lp_filter.getState() - vel_z_prev) / _dt_s);
        local_pos_filtered.az = vel_z_deriv;

    } else {
        local_pos_filtered.vz = NAN;
        local_pos_filtered.az = NAN;

        // reset filters to prevent acceleration spikes when regaining velocity
        _vel_z_lp_filter.reset({});
        _vel_z_notch_filter.reset();
        _vel_deriv_z_lp_filter.reset({});
    }

    const hrt_abstime now = hrt_absolute_time();

    if (_multi_ref_sub.update(&_traj_sp_tmp)) {
        _ref = _traj_sp_tmp;

        if (_ref.horizon_length > multi_trajectory_setpoint_s::MAX_HORIZON) {
            PX4_WARN("clamping invalid horizon_length: %u", _ref.horizon_length);
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

    if (handlePx4DefaultForwarding(now)) {
        return;
    }

    if (_active_type == ControllerType::MPC ||
        _active_type == ControllerType::CMPC ||
        _active_type == ControllerType::DFBC) {

        if (_last_pos_mpc_time == 0 ||
            hrt_elapsed_time(&_last_pos_mpc_time) >= POS_MPC_UPDATE_INTERVAL) {
            _active_controller->computeSetpoints(local_pos_filtered, _ref, _att_sp);
            _last_pos_mpc_time = now;
        }

    } else {
        _active_controller->computeSetpoints(local_pos_filtered, _ref, _att_sp);
    }

    const vehicle_attitude_s attitude_cached = _att_cached;
    adjustAttitudeSetpointForEKFResets(attitude_cached, _att_sp);

    vehicle_angular_velocity_s angular_velocity_for_linearization{};

    if (_last_ang_vel_time != 0 && hrt_elapsed_time(&_last_ang_vel_time) <= ANGULAR_VEL_TIMEOUT) {
        angular_velocity_for_linearization = _ang_vel_cached;

    } else {
        angular_velocity_for_linearization.xyz[0] = NAN;
        angular_velocity_for_linearization.xyz[1] = NAN;
        angular_velocity_for_linearization.xyz[2] = NAN;
    }

    if (_active_type == ControllerType::MPC ||
        _active_type == ControllerType::CMPC) {

        if (_last_att_mpc_time == 0 ||
            hrt_elapsed_time(&_last_att_mpc_time) >= ATT_MPC_UPDATE_INTERVAL) {
            _attitude_mpc_controller.setSuccessiveLinearization(attitude_cached, angular_velocity_for_linearization);
            _attitude_mpc_controller.computeRateSetpoints(attitude_cached, _att_sp, _rate_sp);
            _last_att_mpc_time = now;
        }

    } else {
        if (_last_att_dfbc_time == 0 ||
            hrt_elapsed_time(&_last_att_dfbc_time) >= ATT_DFBC_UPDATE_INTERVAL) {
            _active_controller->computeRateSetpoints(attitude_cached, angular_velocity_for_linearization, _att_sp, _rate_sp);
            _last_att_dfbc_time = now;
        }
    }

    // publish attitude setpoint (for logging/debug) and body-rate setpoint (for control)
    // _att_sp_pub.publish(_att_sp);
    _rate_sp_pub.publish(_rate_sp);
}

void CustomPosControl::publishOffboardControlMode(const hrt_abstime now)
{
    offboard_control_mode_s ocm{};
    ocm.timestamp = now;

    if (_active_type == ControllerType::PX4_DEFAULT || _active_type == ControllerType::SYSID) {
        ocm.position = true;
        ocm.velocity = false;
        ocm.acceleration = false;
        ocm.attitude = false;
        ocm.body_rate = false;

    } else {
        ocm.position = false;
        ocm.velocity = false;
        ocm.acceleration = false;
        ocm.attitude = false;
        ocm.body_rate = true;
    }

    _offboard_control_mode_pub.publish(ocm);
}

bool CustomPosControl::handlePx4DefaultForwarding(const hrt_abstime now)
{
    if (_active_type != ControllerType::PX4_DEFAULT && _active_type != ControllerType::SYSID) {
        return false;
    }

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
    return true;
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

    if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
        _vel_xy_lp_filter.reset(_vel_xy_lp_filter.getState() + matrix::Vector2f(local_pos.delta_vxy));
        _vel_xy_notch_filter.reset();
    }

    if (local_pos.vz_reset_counter != _vz_reset_counter) {
        _vel_z_lp_filter.reset(_vel_z_lp_filter.getState() + local_pos.delta_vz);
        _vel_z_notch_filter.reset();
    }

    // save latest reset counters
    _vxy_reset_counter = local_pos.vxy_reset_counter;
    _vz_reset_counter = local_pos.vz_reset_counter;
    _xy_reset_counter = local_pos.xy_reset_counter;
    _z_reset_counter = local_pos.z_reset_counter;
    _heading_reset_counter = local_pos.heading_reset_counter;
}

void CustomPosControl::adjustAttitudeSetpointForEKFResets(const vehicle_attitude_s &attitude,
                                                          vehicle_attitude_setpoint_s &attitude_setpoint)
{
    if (attitude.quat_reset_counter != _quat_reset_counter) {
        const bool have_older_setpoint = (attitude_setpoint.timestamp != 0) &&
                                         (attitude_setpoint.timestamp < attitude.timestamp);

        if (have_older_setpoint && finiteQuaternion(attitude_setpoint.q_d) && finiteQuaternion(attitude.delta_q_reset)) {
            matrix::Quatf q_setpoint(attitude_setpoint.q_d);
            const matrix::Quatf delta_q_reset(attitude.delta_q_reset);

            q_setpoint = delta_q_reset * q_setpoint;

            const float q_norm = q_setpoint.norm();

            if (PX4_ISFINITE(q_norm) && q_norm > 1e-6f) {
                q_setpoint.normalize();
                q_setpoint.copyTo(attitude_setpoint.q_d);
            }
        }

        _quat_reset_counter = attitude.quat_reset_counter;
    }
}

void CustomPosControl::switchController(ControllerType new_type) {

    if (new_type == _active_type) {
        return;
    }

    delete _active_controller;
    _active_controller = nullptr;

    switch (new_type) {
        case ControllerType::NO_CTRL:
            PX4_WARN("No Controller.");
            break;
        
        case ControllerType::DFBC:
            _active_controller = new DfbcAttitudeController(this);
            _last_pos_mpc_time = 0;
            PX4_INFO("DFBC controller activated.");
            break;
            
        case ControllerType::MPC:
            _active_controller = new MPCPositionController(this);
            _last_pos_mpc_time = 0;
            _last_att_mpc_time = 0;
            PX4_INFO("MPC controller activated.");
            break;
            
        case ControllerType::CMPC:
            _active_controller = new CMPCPositionController(this);
            _last_pos_mpc_time = 0;
            _last_att_mpc_time = 0;
            PX4_WARN("CMPC controller activated.");
            break;

        case ControllerType::PX4_DEFAULT:
            PX4_INFO("PX4 default position controller forwarding activated.");
            break;

        case ControllerType::INDI:
            _active_controller = new IndiAttitudeController(this);
            PX4_INFO("INDI controller activated.");
            break;

        case ControllerType::SYSID:
            PX4_INFO("System identification forwarding controller activated.");
            break;
        }

    _active_type = new_type;

    if (_active_controller) {
        _active_controller->parametersUpdate();
    }
}

int CustomPosControl::task_spawn(int argc, char *argv[])
{
    // Only allow one instance
    if (get_instance() != nullptr) {
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
        return -1;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    PX4_INFO("custom_pos_control started (work queue)");
    return 0;
}

// CLI handling
int CustomPosControl::custom_command(int argc, char *argv[])
{
    if (argc < 1) {
        return print_usage("missing command");
    }

    CustomPosControl *instance = get_instance();

    if (!instance) {
        PX4_WARN("module not running");
        return 0;
    }

   // enable / disable controller
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

    // set controller type
    if (!strcmp(argv[0], "set")) {

        if (argc < 2) {
            return print_usage("set <none|dfbc|mpc|cmpc|px4_default|indi|sysid>");
        }

        int32_t val = -1;

        if (!strcmp(argv[1], "none"))  val = 0;
        if (!strcmp(argv[1], "dfbc"))  val = 1;
        if (!strcmp(argv[1], "mpc"))   val = 2;
        if (!strcmp(argv[1], "cmpc"))  val = 3;
        if (!strcmp(argv[1], "px4_default")) val = 4;
        if (!strcmp(argv[1], "indi")) val = 5;
        if (!strcmp(argv[1], "sysid")) val = 6;

        if (val < 0) {
            PX4_ERR("unknown controller type");
            return 0;
        }

        param_set(param_find("CST_POS_CTRL_TYP"), &val);
        PX4_INFO("controller type set to %d", (int)val);
        return 0;
    }

    return print_usage("unknown command");
}

int CustomPosControl::print_status()
{
    PX4_INFO("Running");
    PX4_INFO("Enabled: %d", (int)_enabled);
    PX4_INFO("Controller type: %d", (int)_active_type);
    PX4_INFO("0: No Controller");
    PX4_INFO("1: DFBC");
    PX4_INFO("2: MPC");
    PX4_INFO("3: CMPC");
    PX4_INFO("4: PX4_DEFAULT");
    PX4_INFO("5: INDI");
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
        Custom position control module with selectable controllers (DFBC / MPC / CMPC / PX4_DEFAULT / INDI / SYSID).
        Works in OFFBOARD mode and publishes body-rate setpoints, or forwards trajectory setpoints to PX4 position
        control when PX4_DEFAULT or SYSID is selected.
        )DESCR_STR");

    PRINT_MODULE_USAGE_NAME("custom_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");

    PRINT_MODULE_USAGE_COMMAND("enable");
    PRINT_MODULE_USAGE_COMMAND("disable");

    PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Select controller");
    PRINT_MODULE_USAGE_ARG("<none|dfbc|mpc|cmpc|px4_default|indi|sysid>", "Controller type", false);

    return 0;
}


extern "C" __EXPORT int custom_pos_control_main(int argc, char *argv[])
{
    return CustomPosControl::main(argc, argv);
}
