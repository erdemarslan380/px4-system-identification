#include "SystemIdentificationLogger.hpp"

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/TopicUtils.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Actuators.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>

#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{
int turningDirectionFromText(const std::string &text)
{
	std::string lowered = text;
	for (char &ch : lowered) {
		ch = static_cast<char>(::tolower(ch));
	}
	return lowered == "ccw" ? 1 : -1;
}

std::string envOr(const char *name, const std::string &fallback = "")
{
	const char *value = std::getenv(name);
	return value ? std::string(value) : fallback;
}

bool matchesScopedName(const std::string &candidate, const std::string &target)
{
	if (candidate == target) {
		return true;
	}
	const std::size_t sep = candidate.rfind("::");
	return sep != std::string::npos && candidate.substr(sep + 2) == target;
}

template<typename ComponentT>
Entity findEntityByNameFallback(const EntityComponentManager &_ecm, const std::string &target)
{
	Entity match{kNullEntity};
	_ecm.Each<components::Name, ComponentT>(
		[&](const Entity &_entity, const components::Name *_name, const ComponentT *) -> bool {
			if (_name && matchesScopedName(_name->Data(), target)) {
				match = _entity;
				return false;
			}
			return true;
		});
	return match;
}
}

namespace px4::simulation
{
struct RotorTelemetryConfig {
	int motor_number{0};
	int turning_direction{-1};
	std::string joint_name;
	std::string link_name;
	double time_constant_up_s{0.0};
	double time_constant_down_s{0.0};
	double max_rot_velocity_radps{0.0};
	double motor_constant{0.0};
	double moment_constant{0.0};
	double rotor_drag_coefficient{0.0};
	double rolling_moment_coefficient{0.0};
	double rotor_velocity_slowdown_sim{1.0};
	Entity joint_entity{kNullEntity};
	Entity link_entity{kNullEntity};
};

struct RotorSample {
	double commanded_radps{NAN};
	double joint_velocity_radps{0.0};
	double actual_velocity_radps{0.0};
	double thrust_n{0.0};
	math::Vector3d drag_force_world{0.0, 0.0, 0.0};
	math::Vector3d drag_basis_world{0.0, 0.0, 0.0};
	math::Vector3d yaw_torque_world{0.0, 0.0, 0.0};
	math::Vector3d rolling_torque_world{0.0, 0.0, 0.0};
	math::Vector3d rolling_basis_world{0.0, 0.0, 0.0};
	math::Vector3d thrust_force_world{0.0, 0.0, 0.0};
	math::Vector3d moment_from_arm_world{0.0, 0.0, 0.0};
};

class SystemIdentificationLoggerPrivate
{
public:
	void OnActuatorMsg(const msgs::Actuators &_msg)
	{
		std::lock_guard<std::mutex> lock(this->msg_mutex);
		this->last_actuators = _msg;
	}

	void EnsureEntities(const EntityComponentManager &_ecm)
	{
		if (!this->base_link_entity_valid) {
			this->base_link_entity = this->model.LinkByName(_ecm, this->base_link_name);
			if (this->base_link_entity == kNullEntity) {
				this->base_link_entity = findEntityByNameFallback<components::Link>(_ecm, this->base_link_name);
			}
			this->base_link_entity_valid = this->base_link_entity != kNullEntity;
		}

		for (auto &rotor : this->rotors) {
			if (rotor.joint_entity == kNullEntity) {
				rotor.joint_entity = this->model.JointByName(_ecm, rotor.joint_name);
				if (rotor.joint_entity == kNullEntity) {
					rotor.joint_entity = findEntityByNameFallback<components::Joint>(_ecm, rotor.joint_name);
				}
			}
			if (rotor.link_entity == kNullEntity) {
				rotor.link_entity = this->model.LinkByName(_ecm, rotor.link_name);
				if (rotor.link_entity == kNullEntity) {
					rotor.link_entity = findEntityByNameFallback<components::Link>(_ecm, rotor.link_name);
				}
			}
		}
	}

	void OpenLog()
	{
		if (!this->enabled || this->stream.is_open() || this->log_path.empty()) {
			return;
		}

		std::filesystem::path path(this->log_path);
		std::filesystem::create_directories(path.parent_path());
		this->stream.open(path, std::ios::out | std::ios::trunc);
		if (!this->stream.is_open()) {
			gzerr << "SystemIdentificationLogger: failed to open log file: " << path << std::endl;
			this->enabled = false;
			return;
		}

		this->stream
			<< "sim_time_us,model_name,truth_mass_kg,truth_ixx_kgm2,truth_iyy_kgm2,truth_izz_kgm2,"
			<< "truth_time_constant_up_s,truth_time_constant_down_s,truth_max_rot_velocity_radps,"
			<< "truth_motor_constant,truth_moment_constant,truth_rotor_drag_coefficient,"
			<< "truth_rolling_moment_coefficient,truth_rotor_velocity_slowdown_sim,"
			<< "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,ax_world_mps2,ay_world_mps2,az_world_mps2,"
			<< "roll,pitch,yaw,p_body,q_body,r_body,p_dot_body,q_dot_body,r_dot_body,"
			<< "total_prop_thrust_n,total_force_body_x_n,total_force_body_y_n,total_force_body_z_n,"
			<< "total_torque_body_x_nm,total_torque_body_y_nm,total_torque_body_z_nm,"
			<< "drag_force_body_x_n,drag_force_body_y_n,drag_force_body_z_n,"
			<< "rolling_moment_body_x_nm,rolling_moment_body_y_nm,rolling_moment_body_z_nm,"
			<< "drag_basis_body_x,drag_basis_body_y,drag_basis_body_z,"
			<< "rolling_basis_body_x,rolling_basis_body_y,rolling_basis_body_z,"
			<< "yaw_moment_basis_n,sum_omega_sq_radps2,sum_command_sq_radps2,sum_actual_rot_velocity_radps,"
			<< "observed_max_rot_velocity_radps,rotor_velocity_slowdown_sim,"
			<< "rotor_0_cmd_radps,rotor_1_cmd_radps,rotor_2_cmd_radps,rotor_3_cmd_radps,"
			<< "rotor_0_joint_vel_radps,rotor_1_joint_vel_radps,rotor_2_joint_vel_radps,rotor_3_joint_vel_radps,"
			<< "rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps,"
			<< "rotor_0_thrust_n,rotor_1_thrust_n,rotor_2_thrust_n,rotor_3_thrust_n\n";
		this->stream.flush();
	}

	std::optional<RotorSample> SampleRotor(const RotorTelemetryConfig &rotor,
					const EntityComponentManager &_ecm,
					const math::Pose3d &base_pose,
					const math::Vector3d &wind_world,
					double commanded_radps) const
	{
		if (rotor.link_entity == kNullEntity) {
			return std::nullopt;
		}

		const auto joint_velocity =
			rotor.joint_entity != kNullEntity ? _ecm.Component<components::JointVelocity>(rotor.joint_entity) : nullptr;
		const auto joint_axis =
			rotor.joint_entity != kNullEntity ? _ecm.Component<components::JointAxis>(rotor.joint_entity) : nullptr;

		Link link(rotor.link_entity);
		const auto world_pose = link.WorldPose(_ecm);
		const auto world_linear_velocity = link.WorldLinearVelocity(_ecm);
		const auto world_angular_velocity = link.WorldAngularVelocity(_ecm);
		if (!world_pose) {
			return std::nullopt;
		}
		const math::Vector3d link_linear_velocity = world_linear_velocity ? *world_linear_velocity : math::Vector3d::Zero;
		const math::Vector3d link_angular_velocity = world_angular_velocity ? *world_angular_velocity : math::Vector3d::Zero;

		RotorSample sample{};
		sample.commanded_radps = commanded_radps;
		math::Vector3d joint_axis_local{0.0, 0.0, 1.0};
		if (joint_axis) {
			joint_axis_local = joint_axis->Data().Xyz();
		}
		const math::Vector3d joint_axis_world = world_pose->Rot().RotateVector(joint_axis_local);
		if (joint_velocity && !joint_velocity->Data().empty()) {
			sample.joint_velocity_radps = joint_velocity->Data()[0];
		} else {
			sample.joint_velocity_radps = link_angular_velocity.Dot(joint_axis_world);
		}
		sample.actual_velocity_radps = std::abs(sample.joint_velocity_radps * rotor.rotor_velocity_slowdown_sim);
		sample.thrust_n = rotor.motor_constant * sample.actual_velocity_radps * sample.actual_velocity_radps;

		const math::Vector3d relative_wind_world = link_linear_velocity - wind_world;
		const math::Vector3d perpendicular_velocity = relative_wind_world
			- (relative_wind_world.Dot(joint_axis_world) * joint_axis_world);
		sample.drag_basis_world = -std::abs(sample.actual_velocity_radps) * perpendicular_velocity;
		sample.drag_force_world = rotor.rotor_drag_coefficient * sample.drag_basis_world;

		const math::Vector3d thrust_local(0.0, 0.0, sample.thrust_n);
		sample.thrust_force_world = world_pose->Rot().RotateVector(thrust_local);
		const math::Vector3d lever_arm_world = world_pose->Pos() - base_pose.Pos();
		sample.moment_from_arm_world = lever_arm_world.Cross(sample.thrust_force_world);

		const math::Vector3d yaw_drag_local(0.0, 0.0, -static_cast<double>(rotor.turning_direction) * sample.thrust_n * rotor.moment_constant);
		const math::Vector3d yaw_drag_body = (base_pose.Inverse() * (*world_pose)).Rot().RotateVector(yaw_drag_local);
		sample.yaw_torque_world = base_pose.Rot().RotateVector(yaw_drag_body);
		sample.rolling_basis_world = -std::abs(sample.actual_velocity_radps) * perpendicular_velocity;
		sample.rolling_torque_world = rotor.rolling_moment_coefficient * sample.rolling_basis_world;
		return sample;
	}

	std::optional<double> CommandForMotor(int motor_number) const
	{
		std::lock_guard<std::mutex> lock(this->msg_mutex);
		if (!this->last_actuators || motor_number < 0 || motor_number >= this->last_actuators->velocity_size()) {
			return std::nullopt;
		}
		return this->last_actuators->velocity(motor_number);
	}

	std::optional<msgs::Actuators> CurrentActuatorsMessage(const EntityComponentManager &_ecm)
	{
		auto actuator_component = _ecm.Component<components::Actuators>(this->model.Entity());
		if (actuator_component) {
			return actuator_component->Data();
		}

		std::lock_guard<std::mutex> lock(this->msg_mutex);
		if (this->last_actuators.has_value()) {
			auto msg = *this->last_actuators;
			this->last_actuators.reset();
			return msg;
		}

		return std::nullopt;
	}

	Model model{kNullEntity};
	std::string model_name;
	std::string robot_namespace;
	std::string base_link_name{"base_link"};
	Entity base_link_entity{kNullEntity};
	bool base_link_entity_valid{false};
	std::string command_sub_topic{"command/motor_speed"};
	std::string log_path;
	double sample_period_s{0.02};
	bool enabled{false};
	std::vector<RotorTelemetryConfig> rotors;
	transport::Node node;
	mutable std::mutex msg_mutex;
	std::optional<msgs::Actuators> last_actuators;
	std::ofstream stream;
	double last_sample_time_s{-1.0};
	math::Vector3d prev_linear_velocity_world{0.0, 0.0, 0.0};
	math::Vector3d prev_body_rates{0.0, 0.0, 0.0};
	math::Pose3d prev_base_pose{};
	bool have_previous_state{false};
	double observed_max_rot_velocity_radps{0.0};
};

SystemIdentificationLogger::SystemIdentificationLogger()
	: dataPtr(std::make_unique<SystemIdentificationLoggerPrivate>())
{
}

SystemIdentificationLogger::~SystemIdentificationLogger() = default;

void SystemIdentificationLogger::Configure(const Entity &_entity,
					   const std::shared_ptr<const sdf::Element> &_sdf,
					   EntityComponentManager &_ecm,
					   EventManager &)
{
	this->dataPtr->model = Model(_entity);
	if (!this->dataPtr->model.Valid(_ecm)) {
		gzerr << "SystemIdentificationLogger must attach to a model entity." << std::endl;
		return;
	}

	this->dataPtr->model_name = this->dataPtr->model.Name(_ecm);
	this->dataPtr->robot_namespace = this->dataPtr->model_name;
	auto sdf_clone = _sdf->Clone();
	const std::string env_dir = envOr("PX4_SYSID_LOG_DIR", "sysid_truth_logs");
	const std::string env_slot = envOr("PX4_SYSID_LOG_SLOT", "manual");
	this->dataPtr->enabled = true;

	if (sdf_clone->HasElement("enabled")) {
		this->dataPtr->enabled = sdf_clone->Get<bool>("enabled") && this->dataPtr->enabled;
	}
	if (sdf_clone->HasElement("base_link_name")) {
		this->dataPtr->base_link_name = sdf_clone->Get<std::string>("base_link_name");
	}
	if (sdf_clone->HasElement("robotNamespace")) {
		this->dataPtr->robot_namespace = sdf_clone->Get<std::string>("robotNamespace");
	}
	if (sdf_clone->HasElement("sample_period_s")) {
		this->dataPtr->sample_period_s = std::max(0.002, sdf_clone->Get<double>("sample_period_s", 0.02).first);
	}
	if (sdf_clone->HasElement("command_sub_topic")) {
		this->dataPtr->command_sub_topic = sdf_clone->Get<std::string>("command_sub_topic");
	}

	std::string configured_log_path;
	if (sdf_clone->HasElement("log_path")) {
		configured_log_path = sdf_clone->Get<std::string>("log_path");
	}
	if (!configured_log_path.empty()) {
		this->dataPtr->log_path = configured_log_path;
	} else if (!env_dir.empty()) {
		const auto boot_us = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		std::ostringstream path;
		path << env_dir << "/" << this->dataPtr->model_name << "_" << env_slot << "_" << boot_us << ".csv";
		this->dataPtr->log_path = path.str();
	}

	gzmsg << "[SystemIdentificationLogger] Configure model=" << this->dataPtr->model_name
	      << " enabled=" << (this->dataPtr->enabled ? "true" : "false")
	      << " log_path=" << this->dataPtr->log_path
	      << " command_topic=" << this->dataPtr->command_sub_topic
	      << std::endl;

	for (auto rotor = sdf_clone->GetElement("rotor"); rotor; rotor = rotor->GetNextElement("rotor")) {
		RotorTelemetryConfig cfg{};
		cfg.motor_number = rotor->Get<int>("motor_number", 0).first;
		cfg.joint_name = rotor->Get<std::string>("joint_name", "").first;
		cfg.link_name = rotor->Get<std::string>("link_name", "").first;
		cfg.turning_direction = turningDirectionFromText(rotor->Get<std::string>("turning_direction", "cw").first);
		cfg.time_constant_up_s = rotor->Get<double>("time_constant_up_s", 0.0).first;
		cfg.time_constant_down_s = rotor->Get<double>("time_constant_down_s", 0.0).first;
		cfg.max_rot_velocity_radps = rotor->Get<double>("max_rot_velocity_radps", 0.0).first;
		cfg.motor_constant = rotor->Get<double>("motor_constant", 0.0).first;
		cfg.moment_constant = rotor->Get<double>("moment_constant", 0.0).first;
		cfg.rotor_drag_coefficient = rotor->Get<double>("rotor_drag_coefficient", 0.0).first;
		cfg.rolling_moment_coefficient = rotor->Get<double>("rolling_moment_coefficient", 0.0).first;
		cfg.rotor_velocity_slowdown_sim = std::max(1e-6, rotor->Get<double>("rotor_velocity_slowdown_sim", 1.0).first);
		if (!cfg.joint_name.empty() && !cfg.link_name.empty()) {
			this->dataPtr->rotors.push_back(cfg);
		}
	}

	gzmsg << "[SystemIdentificationLogger] Rotor configs: " << this->dataPtr->rotors.size() << std::endl;

	if (this->dataPtr->enabled && !this->dataPtr->rotors.empty()) {
		const std::string topic = transport::TopicUtils::AsValidTopic(
			this->dataPtr->robot_namespace + "/" + this->dataPtr->command_sub_topic);
		this->dataPtr->node.Subscribe(topic,
			&SystemIdentificationLoggerPrivate::OnActuatorMsg,
			this->dataPtr.get());
		gzmsg << "[SystemIdentificationLogger] Subscribed to " << topic << std::endl;
		this->dataPtr->OpenLog();
	} else if (!this->dataPtr->enabled) {
		gzwarn << "[SystemIdentificationLogger] Disabled by SDF configuration." << std::endl;
	} else {
		gzwarn << "[SystemIdentificationLogger] No rotor configs found; logger will stay idle." << std::endl;
	}
}

void SystemIdentificationLogger::PreUpdate(const UpdateInfo &_info,
					  EntityComponentManager &_ecm)
{
	GZ_PROFILE("SystemIdentificationLogger::PreUpdate");
	if (!this->dataPtr->enabled || _info.paused || this->dataPtr->rotors.empty()) {
		return;
	}

	this->dataPtr->EnsureEntities(_ecm);
	if (!this->dataPtr->base_link_entity_valid) {
		static bool warned_once = false;
		if (!warned_once) {
			gzwarn << "[SystemIdentificationLogger] Base link not resolved yet for model "
			       << this->dataPtr->model_name << " expected link=" << this->dataPtr->base_link_name << std::endl;
			warned_once = true;
		}
		return;
	}

	Link base_link(this->dataPtr->base_link_entity);
	const auto base_pose = base_link.WorldPose(_ecm);
	const auto base_linear_velocity = base_link.WorldLinearVelocity(_ecm);
	const auto base_angular_velocity = base_link.WorldAngularVelocity(_ecm);
	const auto base_inertial = _ecm.Component<components::Inertial>(this->dataPtr->base_link_entity);
	if (!base_pose) {
		return;
	}
	const math::Vector3d linear_velocity_world =
		base_linear_velocity ? *base_linear_velocity : math::Vector3d::Zero;
	math::Vector3d angular_velocity_world =
		base_angular_velocity ? *base_angular_velocity : math::Vector3d::Zero;

	const double sim_time_s = std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count();
	if (this->dataPtr->last_sample_time_s >= 0.0 && (sim_time_s - this->dataPtr->last_sample_time_s) < (this->dataPtr->sample_period_s - 1e-6)) {
		return;
	}

	this->dataPtr->OpenLog();
	if (!this->dataPtr->stream.is_open()) {
		return;
	}

	math::Vector3d wind_world{0.0, 0.0, 0.0};
	const Entity wind_entity = _ecm.EntityByComponents(components::Wind());
	if (wind_entity != kNullEntity) {
		const auto wind_linear_velocity = _ecm.Component<components::WorldLinearVelocity>(wind_entity);
		if (wind_linear_velocity) {
			wind_world = wind_linear_velocity->Data();
		}
	}

	math::Vector3d linear_accel_world{0.0, 0.0, 0.0};
	math::Vector3d body_rate_accel{0.0, 0.0, 0.0};
	if (this->dataPtr->have_previous_state && sim_time_s > this->dataPtr->last_sample_time_s) {
		const double dt = sim_time_s - this->dataPtr->last_sample_time_s;
		const math::Quaterniond dq = this->dataPtr->prev_base_pose.Rot().Inverse() * base_pose->Rot();
		math::Vector3d dq_axis{1.0, 0.0, 0.0};
		double dq_angle = 0.0;
		dq.AxisAngle(dq_axis, dq_angle);
		if (std::isfinite(dq_angle) && std::abs(dq_angle) > 1e-9) {
			if (dq_axis.Length() > 1e-9) {
				const math::Vector3d derived_body_rates = dq_axis.Normalized() * (dq_angle / dt);
				if (angular_velocity_world.Length() < 1e-6) {
					angular_velocity_world = base_pose->Rot().RotateVector(derived_body_rates);
				}
			}
		}
		linear_accel_world = (linear_velocity_world - this->dataPtr->prev_linear_velocity_world) / dt;
		const math::Vector3d current_body_rates = base_pose->Rot().Inverse().RotateVector(angular_velocity_world);
		body_rate_accel = (current_body_rates - this->dataPtr->prev_body_rates) / dt;
		this->dataPtr->prev_body_rates = current_body_rates;
	} else {
		this->dataPtr->prev_body_rates = base_pose->Rot().Inverse().RotateVector(angular_velocity_world);
	}
	this->dataPtr->prev_linear_velocity_world = linear_velocity_world;
	this->dataPtr->prev_base_pose = *base_pose;
	this->dataPtr->last_sample_time_s = sim_time_s;
	this->dataPtr->have_previous_state = true;
	const math::Vector3d body_rates = base_pose->Rot().Inverse().RotateVector(angular_velocity_world);

	math::Vector3d total_force_world{0.0, 0.0, 0.0};
	math::Vector3d total_torque_world{0.0, 0.0, 0.0};
	math::Vector3d drag_force_world{0.0, 0.0, 0.0};
	math::Vector3d rolling_torque_world{0.0, 0.0, 0.0};
	math::Vector3d drag_basis_world{0.0, 0.0, 0.0};
	math::Vector3d rolling_basis_world{0.0, 0.0, 0.0};
	double total_prop_thrust_n = 0.0;
	double yaw_moment_basis_n = 0.0;
	double sum_omega_sq = 0.0;
	double sum_command_sq = 0.0;
	double sum_actual_velocity = 0.0;
	double truth_time_constant_up_s = NAN;
	double truth_time_constant_down_s = NAN;
	double truth_max_rot_velocity_radps = NAN;
	double truth_motor_constant = NAN;
	double truth_moment_constant = NAN;
	double truth_rotor_drag_coefficient = NAN;
	double truth_rolling_moment_coefficient = NAN;
	double truth_rotor_velocity_slowdown_sim = NAN;
	double truth_mass_kg = NAN;
	double truth_ixx_kgm2 = NAN;
	double truth_iyy_kgm2 = NAN;
	double truth_izz_kgm2 = NAN;
	RotorSample rotor_samples[4]{};
	const auto actuators_msg = this->dataPtr->CurrentActuatorsMessage(_ecm);

	if (!this->dataPtr->rotors.empty()) {
		double sum_time_constant_up_s = 0.0;
		double sum_time_constant_down_s = 0.0;
		double sum_max_rot_velocity_radps = 0.0;
		double sum_motor_constant = 0.0;
		double sum_moment_constant = 0.0;
		double sum_rotor_drag_coefficient = 0.0;
		double sum_rolling_moment_coefficient = 0.0;
		double sum_rotor_velocity_slowdown_sim = 0.0;
		for (const auto &rotor : this->dataPtr->rotors) {
			sum_time_constant_up_s += rotor.time_constant_up_s;
			sum_time_constant_down_s += rotor.time_constant_down_s;
			sum_max_rot_velocity_radps += rotor.max_rot_velocity_radps;
			sum_motor_constant += rotor.motor_constant;
			sum_moment_constant += rotor.moment_constant;
			sum_rotor_drag_coefficient += rotor.rotor_drag_coefficient;
			sum_rolling_moment_coefficient += rotor.rolling_moment_coefficient;
			sum_rotor_velocity_slowdown_sim += rotor.rotor_velocity_slowdown_sim;
		}
		const double rotor_count = static_cast<double>(this->dataPtr->rotors.size());
		truth_time_constant_up_s = sum_time_constant_up_s / rotor_count;
		truth_time_constant_down_s = sum_time_constant_down_s / rotor_count;
		truth_max_rot_velocity_radps = sum_max_rot_velocity_radps / rotor_count;
		truth_motor_constant = sum_motor_constant / rotor_count;
		truth_moment_constant = sum_moment_constant / rotor_count;
		truth_rotor_drag_coefficient = sum_rotor_drag_coefficient / rotor_count;
		truth_rolling_moment_coefficient = sum_rolling_moment_coefficient / rotor_count;
		truth_rotor_velocity_slowdown_sim = sum_rotor_velocity_slowdown_sim / rotor_count;
	}

	if (base_inertial) {
		const auto &inertial = base_inertial->Data();
		truth_mass_kg = inertial.MassMatrix().Mass();
		const auto moi = inertial.Moi();
		truth_ixx_kgm2 = moi(0, 0);
		truth_iyy_kgm2 = moi(1, 1);
		truth_izz_kgm2 = moi(2, 2);
	}

	for (const auto &rotor : this->dataPtr->rotors) {
		std::optional<double> commanded;
		if (actuators_msg.has_value() &&
		    rotor.motor_number >= 0 &&
		    rotor.motor_number < actuators_msg->velocity_size()) {
			commanded = actuators_msg->velocity(rotor.motor_number);
		} else {
			commanded = this->dataPtr->CommandForMotor(rotor.motor_number);
		}
		const auto sample = this->dataPtr->SampleRotor(rotor, _ecm, *base_pose, wind_world, commanded.value_or(NAN));
		if (!sample.has_value()) {
			continue;
		}
		const RotorSample &value = *sample;
		if (rotor.motor_number >= 0 && rotor.motor_number < 4) {
			rotor_samples[rotor.motor_number] = value;
		}
		total_prop_thrust_n += value.thrust_n;
		total_force_world += value.thrust_force_world + value.drag_force_world;
		total_torque_world += value.moment_from_arm_world + value.yaw_torque_world + value.rolling_torque_world;
		drag_force_world += value.drag_force_world;
		rolling_torque_world += value.rolling_torque_world;
		drag_basis_world += value.drag_basis_world;
		rolling_basis_world += value.rolling_basis_world;
		yaw_moment_basis_n += -static_cast<double>(rotor.turning_direction) * value.thrust_n;
		sum_omega_sq += value.actual_velocity_radps * value.actual_velocity_radps;
		sum_actual_velocity += value.actual_velocity_radps;
		if (std::isfinite(value.commanded_radps)) {
			sum_command_sq += value.commanded_radps * value.commanded_radps;
		}
		this->dataPtr->observed_max_rot_velocity_radps = std::max(this->dataPtr->observed_max_rot_velocity_radps, value.actual_velocity_radps);
	}

	const math::Vector3d total_force_body = base_pose->Rot().Inverse().RotateVector(total_force_world);
	const math::Vector3d total_torque_body = base_pose->Rot().Inverse().RotateVector(total_torque_world);
	const math::Vector3d drag_force_body = base_pose->Rot().Inverse().RotateVector(drag_force_world);
	const math::Vector3d rolling_torque_body = base_pose->Rot().Inverse().RotateVector(rolling_torque_world);
	const math::Vector3d drag_basis_body = base_pose->Rot().Inverse().RotateVector(drag_basis_world);
	const math::Vector3d rolling_basis_body = base_pose->Rot().Inverse().RotateVector(rolling_basis_world);
	const math::Vector3d euler = base_pose->Rot().Euler();
	const auto sim_time_us = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count();

	this->dataPtr->stream << std::scientific << std::setprecision(12)
		<< sim_time_us << ','
		<< this->dataPtr->model_name << ','
		<< truth_mass_kg << ',' << truth_ixx_kgm2 << ',' << truth_iyy_kgm2 << ',' << truth_izz_kgm2 << ','
		<< truth_time_constant_up_s << ',' << truth_time_constant_down_s << ',' << truth_max_rot_velocity_radps << ','
		<< truth_motor_constant << ',' << truth_moment_constant << ',' << truth_rotor_drag_coefficient << ','
		<< truth_rolling_moment_coefficient << ',' << truth_rotor_velocity_slowdown_sim << ','
		<< std::fixed << std::setprecision(6)
		<< base_pose->Pos().X() << ',' << base_pose->Pos().Y() << ',' << base_pose->Pos().Z() << ','
		<< linear_velocity_world.X() << ',' << linear_velocity_world.Y() << ',' << linear_velocity_world.Z() << ','
		<< linear_accel_world.X() << ',' << linear_accel_world.Y() << ',' << linear_accel_world.Z() << ','
		<< euler.X() << ',' << euler.Y() << ',' << euler.Z() << ','
		<< body_rates.X() << ',' << body_rates.Y() << ',' << body_rates.Z() << ','
		<< body_rate_accel.X() << ',' << body_rate_accel.Y() << ',' << body_rate_accel.Z() << ','
		<< total_prop_thrust_n << ','
		<< total_force_body.X() << ',' << total_force_body.Y() << ',' << total_force_body.Z() << ','
		<< total_torque_body.X() << ',' << total_torque_body.Y() << ',' << total_torque_body.Z() << ','
		<< drag_force_body.X() << ',' << drag_force_body.Y() << ',' << drag_force_body.Z() << ','
		<< rolling_torque_body.X() << ',' << rolling_torque_body.Y() << ',' << rolling_torque_body.Z() << ','
		<< drag_basis_body.X() << ',' << drag_basis_body.Y() << ',' << drag_basis_body.Z() << ','
		<< rolling_basis_body.X() << ',' << rolling_basis_body.Y() << ',' << rolling_basis_body.Z() << ','
		<< yaw_moment_basis_n << ',' << sum_omega_sq << ',' << sum_command_sq << ',' << sum_actual_velocity << ','
		<< this->dataPtr->observed_max_rot_velocity_radps << ','
		<< (this->dataPtr->rotors.empty() ? 1.0 : this->dataPtr->rotors.front().rotor_velocity_slowdown_sim);

	for (int rotor_index = 0; rotor_index < 4; ++rotor_index) {
		this->dataPtr->stream << ',' << rotor_samples[rotor_index].commanded_radps;
	}
	for (int rotor_index = 0; rotor_index < 4; ++rotor_index) {
		this->dataPtr->stream << ',' << rotor_samples[rotor_index].joint_velocity_radps;
	}
	for (int rotor_index = 0; rotor_index < 4; ++rotor_index) {
		this->dataPtr->stream << ',' << rotor_samples[rotor_index].actual_velocity_radps;
	}
	for (int rotor_index = 0; rotor_index < 4; ++rotor_index) {
		this->dataPtr->stream << ',' << rotor_samples[rotor_index].thrust_n;
	}
	this->dataPtr->stream << '\n';
	this->dataPtr->stream.flush();
}
} // namespace px4::simulation

GZ_ADD_PLUGIN(
	px4::simulation::SystemIdentificationLogger,
	gz::sim::System,
	px4::simulation::SystemIdentificationLogger::ISystemConfigure,
	px4::simulation::SystemIdentificationLogger::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(px4::simulation::SystemIdentificationLogger, "SystemIdentificationLoggerPlugin")
