#pragma once

#include <matrix/matrix/math.hpp>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multi_trajectory_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_hrt.h>

using namespace time_literals;

enum class ControllerType : uint8_t {
	NO_CTRL = 0,
	PX4_DEFAULT = 4,
	SYSID = 6,
};

class CustomPosControl :
	public ModuleBase<CustomPosControl>,
	public px4::ScheduledWorkItem,
	public ModuleParams
{
public:
	CustomPosControl();
	~CustomPosControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool init();

private:
	void Run() override;
	void parametersUpdate();
	void switchController(ControllerType new_type);
	void updateRcControllerSelection();
	void publishOffboardControlMode(const hrt_abstime now);
	void publishForwardedSetpoint(const hrt_abstime now);
	void adjustSetpointForEKFResets(const vehicle_local_position_s &local_pos,
					  multi_trajectory_setpoint_s &setpoint);
	bool isActiveForwardingController() const;

	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _multi_ref_sub{ORB_ID(multi_trajectory_setpoint)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<trajectory_setpoint_s> _trajectory_sp_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

	int32_t _enabled{0};
	uint8_t _controller_type{0};
	ControllerType _active_type{ControllerType::NO_CTRL};
	int32_t _rc_select_enabled{0};
	int32_t _rc_controller_channel{0};
	int32_t _rc_selected_slot{-1};

	vehicle_local_position_s _local_pos_cached{};
	hrt_abstime _last_local_pos_time{0};
	static constexpr hrt_abstime LOCAL_POS_TIMEOUT = 200_ms;
	static constexpr hrt_abstime REF_TIMEOUT = 200_ms;

	multi_trajectory_setpoint_s _traj_sp_tmp{};
	multi_trajectory_setpoint_s _ref{};
	hrt_abstime _last_ref_time{0};

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CST_POS_CTRL_EN>) _param_cst_pos_en,
		(ParamInt<px4::params::CST_POS_CTRL_TYP>) _param_cst_pos_typ,
		(ParamInt<px4::params::CST_RC_SEL_EN>) _param_cst_rc_sel_en,
		(ParamInt<px4::params::CST_RC_CTRL_CH>) _param_cst_rc_ctrl_ch
	)
};
