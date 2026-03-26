// custom_pos_control.hpp
// A module to implement custom position controllers
// currently we will subscribe to: /vehicle_local_position, /vehicle_attitude, /vehicle_angular_velocity,
// /vehicle_status, /parameter_update, and custom /multi_trajectory_setpoint and /hover_thrust_estimate
// we publish to /offboard_control_mode, /vehicle_attitude_setpoint, /vehicle_rates_setpoint and /trajectory_setpoint
// current our mode only works in offboard mode
// this is not ideal we will switch back to another method later
#pragma once

#include "dfbc_attitude_controller.hpp"
#include "indi_attitude_controller.hpp"
#include "mpc_pos_controller.hpp"
#include "cmpc_pos_controller.hpp"
#include <mpc_attitude_controller.hpp>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/defines.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

// uORB messages
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/multi_trajectory_setpoint.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <matrix/matrix/math.hpp>

#include <drivers/drv_hrt.h>
using namespace time_literals;

#include <cstdint>

enum class ControllerType : uint8_t {
    NO_CTRL = 0,
    DFBC = 1,
    MPC = 2,
    CMPC = 3,
    PX4_DEFAULT = 4,
    INDI = 5,
    SYSID = 6
};

class CustomPosControl :
    public ModuleBase<CustomPosControl>,
    public px4::ScheduledWorkItem,
    public ModuleParams
{
public:
    CustomPosControl();
    ~CustomPosControl();

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    int print_status() override;
    static int print_usage(const char *reason = nullptr);
    bool init();

private:
    void Run() override;
    void parametersUpdate();
    void switchController(ControllerType new_type);
    void updateRcControllerSelection();
    void publishOffboardControlMode(const hrt_abstime now);
    bool handlePx4DefaultForwarding(const hrt_abstime now);
    void adjustSetpointForEKFResets(const vehicle_local_position_s &local_pos,
                                    multi_trajectory_setpoint_s &setpoint);
    void adjustAttitudeSetpointForEKFResets(const vehicle_attitude_s &attitude,
                                            vehicle_attitude_setpoint_s &attitude_setpoint);

    // publishers and subscriber
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _ang_vel_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription _multi_ref_sub{ORB_ID(multi_trajectory_setpoint)};
    uORB::Subscription _hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
    uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
    uORB::Publication<vehicle_attitude_setpoint_s>
        _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Publication<vehicle_rates_setpoint_s>
        _rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Publication<trajectory_setpoint_s>
        _trajectory_sp_pub{ORB_ID(trajectory_setpoint)};
    uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

    int32_t _enabled{0};

    // parameter for controller selection
    // @value 1: DFBC
    // @value 2: MPC
    // @value 3: CMPC
    // @value 4: PX4_DEFAULT
    // @value 5: INDI
    uint8_t _controller_type{0}; // default NO_CTRL
    ControllerType _active_type{ControllerType::NO_CTRL};
    int32_t _rc_select_enabled{0};
    int32_t _rc_controller_channel{0};
    int32_t _rc_selected_slot{-1};

    // local position caching
    vehicle_local_position_s _local_pos_cached{};
    hrt_abstime _last_local_pos_time{0};
    vehicle_attitude_s _att_cached{};
    hrt_abstime _last_att_time{0};
    vehicle_angular_velocity_s _ang_vel_cached{};
    hrt_abstime _last_ang_vel_time{0};
    static constexpr hrt_abstime LOCAL_POS_TIMEOUT = 200_ms;
    static constexpr hrt_abstime ATTITUDE_TIMEOUT = 200_ms;
    static constexpr hrt_abstime ANGULAR_VEL_TIMEOUT = 200_ms;
    static constexpr hrt_abstime REF_TIMEOUT = 200_ms;
    hrt_abstime _time_stamp_last_loop{0};
    math::WelfordMean<float> _sample_interval_s{};
    float _dt_s{0.004f};
    // attitude setpoint
    vehicle_attitude_setpoint_s _att_sp{};
    // body rate setpoint
    vehicle_rates_setpoint_s _rate_sp{};

    // position controller update interval (MPC/CMPC/DFBC)
    static constexpr hrt_abstime POS_MPC_UPDATE_INTERVAL{20_ms};
    hrt_abstime _last_pos_mpc_time{0};
    // attitude MPC update interval (MPC/CMPC)
    static constexpr hrt_abstime ATT_MPC_UPDATE_INTERVAL{4_ms};
    hrt_abstime _last_att_mpc_time{0};
    // attitude DFBC update interval (DFBC)
    static constexpr hrt_abstime ATT_DFBC_UPDATE_INTERVAL{4_ms};
    hrt_abstime _last_att_dfbc_time{0};
    

    // velocity filters
    math::NotchFilter<matrix::Vector2f> _vel_xy_notch_filter{};
    math::NotchFilter<float> _vel_z_notch_filter{};
    AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};
    AlphaFilter<float> _vel_z_lp_filter{};
    AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};
    AlphaFilter<float> _vel_deriv_z_lp_filter{};

    // EKF reset counters
    uint8_t _vxy_reset_counter{0};
    uint8_t _vz_reset_counter{0};
    uint8_t _xy_reset_counter{0};
    uint8_t _z_reset_counter{0};
    uint8_t _heading_reset_counter{0};
    uint8_t _quat_reset_counter{0};

    // parameters
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::CST_POS_CTRL_EN>) _param_cst_pos_en,
        (ParamInt<px4::params::CST_POS_CTRL_TYP>) _param_cst_pos_typ,
        (ParamInt<px4::params::CST_RC_SEL_EN>) _param_cst_rc_sel_en,
        (ParamInt<px4::params::CST_RC_CTRL_CH>) _param_cst_rc_ctrl_ch,
        (ParamFloat<px4::params::MPC_VEL_NF_FRQ>) _param_mpc_vel_nf_frq,
        (ParamFloat<px4::params::MPC_VEL_NF_BW>) _param_mpc_vel_nf_bw,
        (ParamFloat<px4::params::MPC_VEL_LP>) _param_mpc_vel_lp,
        (ParamFloat<px4::params::MPC_VELD_LP>) _param_mpc_veld_lp
    )

    // pointer to a controller
    CustomPosController *_active_controller{nullptr};
    MPCAttitudeController _attitude_mpc_controller{this};
    
    // reference
    multi_trajectory_setpoint_s _traj_sp_tmp{};
    multi_trajectory_setpoint_s _ref{};
    hrt_abstime _last_ref_time{0};
};
