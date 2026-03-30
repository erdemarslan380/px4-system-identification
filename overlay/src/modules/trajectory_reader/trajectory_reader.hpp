/**
 * @file trajectory_reader.hpp
 *
 * TrajectoryReader streams preprocessed binary trajectory files (.traj)
 * from persistent storage and publishes multi-step trajectory setpoints
 * via uORB for downstream controllers (e.g. MPC).
 *
 * The binary format is fixed-layout and optimized for deterministic
 * runtime behavior on embedded systems.
 */

#pragma once

#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/multi_trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/hover_thrust_estimate.h>

#include <matrix/matrix/math.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

struct TrajSample {
	float px, py, pz;
	float vx, vy, vz;
	float ax, ay, az;
	float yaw, yawspeed;
};

enum class Mode : uint8_t {
	POSITION = 0,
	TRAJECTORY = 1,
	IDENTIFICATION = 2
};

enum class IdentificationProfile : uint8_t {
	HOVER_THRUST = 0,
	ROLL_SWEEP = 1,
	PITCH_SWEEP = 2,
	YAW_SWEEP = 3,
	DRAG_X = 4,
	DRAG_Y = 5,
	DRAG_Z = 6,
	MASS_VERTICAL = 7,
	MOTOR_STEP = 8
};

enum class CampaignType : uint8_t {
	NONE = 0,
	IDENTIFICATION_ONLY = 1,
	FULL_STACK = 2,
	TRAJECTORY_ONLY = 3
};

enum class CampaignStage : uint8_t {
	IDLE = 0,
	RETURN_TO_ANCHOR = 1,
	RUN_SEGMENT = 2,
	COMPLETE = 3
};

class TrajectoryReader :
	public ModuleBase,
	public px4::ScheduledWorkItem,
	public ModuleParams
{
public:
	static Descriptor desc;

	TrajectoryReader();
	~TrajectoryReader() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

	void setTrajectoryReaderMode(Mode mode);
	bool setTrajectoryId(uint8_t traj_id);
	void setPositionModeRef(const matrix::Vector3f &pos, float yaw, bool absolute = false);
	void setTrajectoryAnchor(const matrix::Vector3f &pos);

    // Setting all parameters from the outside saves 300bytes flash
	void setParamMpcAccHor(const float param_mpc_acc_hor) { _mpc_acc_hor = param_mpc_acc_hor; }
	void setParamMpcAccDownMax(const float param_mpc_acc_down_max) { _mpc_acc_down_max = param_mpc_acc_down_max; }
	void setParamMpcAccUpMax(const float param_mpc_acc_up_max) { _mpc_acc_up_max = param_mpc_acc_up_max; }
	void setParamMpcJerkAuto(const float param_mpc_jerk_auto) { _position_smoothing.setMaxJerk(param_mpc_jerk_auto); }
	void setParamMpcXyCruise(const float param_mpc_xy_cruise) { _mpc_xy_cruise = param_mpc_xy_cruise; }
	void setParamMpcXyErrMax(const float param_mpc_xy_err_max) { _position_smoothing.setMaxAllowedHorizontalError(param_mpc_xy_err_max); }
	void setParamMpcXyVelMax(const float param_mpc_xy_vel_max) { _position_smoothing.setMaxVelocityXY(param_mpc_xy_vel_max); }
	void setParamMpcYawrautoMax(const float param_mpc_yawrauto_max) { _mpc_yawrauto_max = param_mpc_yawrauto_max; }
	void setParamMpcYawrautoAcc(const float param_mpc_yawrauto_acc) { _mpc_yawrauto_acc = param_mpc_yawrauto_acc; }
	void setParamMpcZVAutoDn(const float param_mpc_z_v_auto_dn) { _mpc_z_v_auto_dn = param_mpc_z_v_auto_dn; }
	void setParamMpcZVAutoUp(const float param_mpc_z_v_auto_up) { _mpc_z_v_auto_up = param_mpc_z_v_auto_up; }

private:
	void Run() override;
	void parametersUpdate();

	// file handling
	bool openTrajectoryFile();
	void closeTrajectoryFile();
	bool storageRootAvailable() const;
	bool loadNextHorizon();
	void publishSetpoint(const matrix::Vector3f &current_pos);
	void publishHoldPositionSetpoint(const matrix::Vector3f &current_pos);
	bool startTrajectoryTrackingLog(hrt_abstime setpoint_timestamp);
	void stopTrajectoryTrackingLog();
	void queueTrackingLogSample(const matrix::Vector3f &ref_first_pos, hrt_abstime setpoint_timestamp);
	void flushTrackingLogSamples(const matrix::Vector3f &current_pos, hrt_abstime now);
	void clearPendingTrackingSamples();
	void clearTrackingLogFailure();
	void syncTrackingLogIfNeeded(bool force);
	void syncIdentificationLogIfNeeded(bool force);
	bool requestCampaignStart();
	void stopCampaign(bool announce);
	bool startCampaign(const matrix::Vector3f &current_pos, hrt_abstime now);
	void updateCampaign(const matrix::Vector3f &current_pos, hrt_abstime now, bool in_offboard);
	bool startCampaignSegment(size_t item_index, hrt_abstime now);
	bool campaignSegmentCompleted() const;
	void transitionCampaignToAnchor(const matrix::Vector3f &current_pos, hrt_abstime now, bool announce);
	const char *campaignTypeToString(CampaignType type) const;
	void setCampaignStatusParam(int32_t status);
	void updateControllerTypeCache();
	const char *controllerTypeToString(int32_t controller_type) const;
	const char *identProfileToString(IdentificationProfile profile) const;
	const char *identProfilePurpose(IdentificationProfile profile) const;
	float identificationDurationS(IdentificationProfile profile) const;
	void updateRcSelections();
	void resetIdentificationState();
	bool startIdentificationLog(hrt_abstime setpoint_timestamp);
	void stopIdentificationLog();
	void writeIdentificationLogSample(const matrix::Vector3f &current_pos, hrt_abstime now);
	void fillIdentificationBuffer(const matrix::Vector3f &current_pos, hrt_abstime now);
	matrix::Vector3f identificationRelativePosition(IdentificationProfile profile, float t_s) const;
	float identificationRelativeYaw(IdentificationProfile profile, float t_s) const;
	float identificationRelativeYawRate(IdentificationProfile profile, float t_s) const;
	void identificationKinematics(
		IdentificationProfile profile,
		float t_s,
		matrix::Vector3f &position,
		matrix::Vector3f &velocity,
		matrix::Vector3f &acceleration,
		float &yaw,
		float &yawspeed) const;

	// smoothing helpers
	void resetSmoothers(const matrix::Vector3f &pos, float yaw);
	void setPositionSmootherLimits(const goto_setpoint_s &sp);
	void setHeadingSmootherLimits(const goto_setpoint_s &sp);

	static constexpr size_t MAX_HORIZON = 20;

	// uORB
	uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _attitude_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Publication<multi_trajectory_setpoint_s> _pub{ORB_ID(multi_trajectory_setpoint)};

	// smoothers
	PositionSmoothing _position_smoothing;
	HeadingSmoothing _heading_smoothing;

	// buffer
	TrajSample _buffer[MAX_HORIZON]{};
	size_t _buffer_len{0};

	// file state
	int _fd{-1};
	uint8_t _traj_id{0};
	uint32_t _num_samples{0};
	uint32_t _cursor{0};
	bool _eof{false};

	// trajectory tracking logging state
	struct PendingTrackingSample {
		hrt_abstime setpoint_timestamp{0};
		matrix::Vector3f ref_first_pos{};
		int32_t controller_type{0};
	};

	static constexpr size_t MAX_PENDING_TRACKING_SAMPLES = 32;
	static constexpr hrt_abstime TRACKING_COMPARISON_DELAY_US{200000}; // 200 ms
	static constexpr uint8_t LOG_FSYNC_INTERVAL_SAMPLES{10};
	static constexpr float CAMPAIGN_RETURN_RADIUS_M{0.35f};
	static constexpr hrt_abstime CAMPAIGN_RETURN_SETTLE_US{1000000};
	PendingTrackingSample _pending_tracking_samples[MAX_PENDING_TRACKING_SAMPLES]{};
	size_t _pending_tracking_head{0};
	size_t _pending_tracking_size{0};
	int _tracking_log_fd{-1};
	bool _start_new_tracking_log{false};
	bool _finalize_tracking_log{false};
	uint16_t _tracking_log_run_counter{0};
	uint8_t _tracking_log_samples_since_sync{0};
	int32_t _controller_type_cached{0};
	bool _tracking_log_failed_latched{false};
	uint32_t _tracking_log_fail_count{0};
	int _ident_log_fd{-1};
	uint16_t _ident_log_run_counter{0};
	uint8_t _ident_log_samples_since_sync{0};

	// mode & state
	Mode _mode{Mode::POSITION};
	IdentificationProfile _ident_profile{IdentificationProfile::HOVER_THRUST};
	vehicle_status_s _status{};
	uint8_t _prev_nav_state{vehicle_status_s::NAVIGATION_STATE_MAX};
	bool _need_traj_offset{false};
	bool _traj_offset_valid{false};
	matrix::Vector3f _traj_offset{};
	bool _traj_anchor_valid{false};
	matrix::Vector3f _traj_anchor{};

	bool _need_pos_offset{false};
	bool _pos_offset_valid{false};
	matrix::Vector3f _pos_offset{};
	bool _pos_ref_absolute{false};
	bool _ident_origin_valid{false};
	matrix::Vector3f _ident_origin{};
	hrt_abstime _ident_start_time{0};
	bool _ident_started{false};
	bool _ident_finalize_log{false};
	bool _ident_start_announced{false};
	bool _ident_completion_announced{false};
	int32_t _rc_select_enabled{0};
	int32_t _rc_selector_channel{0};
	int32_t _rc_selector_max_traj_id{3};
	int32_t _rc_selected_index{-1};
	int32_t _param_mode_cmd_cached{-1};
	int32_t _param_traj_id_cached{-1};
	int32_t _param_ident_profile_cached{-1};
	int32_t _param_campaign_cached{-1};
	int32_t _param_campaign_cmd_cached{-1};
	bool _param_anchor_cached_valid{false};
	matrix::Vector3f _param_anchor_cached{};
	CampaignType _campaign_type{CampaignType::NONE};
	CampaignStage _campaign_stage{CampaignStage::IDLE};
	bool _campaign_active{false};
	bool _campaign_start_requested{false};
	size_t _campaign_item_index{0};
	matrix::Vector3f _campaign_anchor{};
	float _campaign_yaw{0.f};
	hrt_abstime _campaign_anchor_settle_since{0};
	hrt_abstime _campaign_last_transition_time{0};

	// position mode
	matrix::Vector3f _pos_target{};
	float _pos_yaw{0.f};
	bool _need_reset{true};

	// parameters (injected externally or via PX4 params)
	float _mpc_acc_hor{0.f};
	float _mpc_xy_cruise{0.f};
	float _mpc_z_v_auto_up{0.f};
	float _mpc_z_v_auto_dn{0.f};
	float _mpc_acc_up_max{0.f};
	float _mpc_acc_down_max{0.f};
	float _mpc_yawrauto_max{0.f};
	float _mpc_yawrauto_acc{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,
		(ParamInt<px4::params::TRJ_RC_SEL_EN>) _param_trj_rc_sel_en,
		(ParamInt<px4::params::TRJ_RC_SEL_CH>) _param_trj_rc_sel_ch,
		(ParamInt<px4::params::TRJ_RC_MAX_ID>) _param_trj_rc_max_id,
		(ParamInt<px4::params::TRJ_IDENT_PROF>) _param_trj_ident_prof,
		(ParamInt<px4::params::TRJ_CAMPAIGN>) _param_trj_campaign,
		(ParamInt<px4::params::TRJ_CAMPAIGN_CMD>) _param_trj_campaign_cmd,
		(ParamInt<px4::params::TRJ_CAMPAIGN_STA>) _param_trj_campaign_sta,
		(ParamInt<px4::params::TRJ_MODE_CMD>) _param_trj_mode_cmd,
		(ParamInt<px4::params::TRJ_ACTIVE_ID>) _param_trj_active_id,
		(ParamFloat<px4::params::TRJ_ANCHOR_X>) _param_trj_anchor_x,
		(ParamFloat<px4::params::TRJ_ANCHOR_Y>) _param_trj_anchor_y,
		(ParamFloat<px4::params::TRJ_ANCHOR_Z>) _param_trj_anchor_z
	)
};
