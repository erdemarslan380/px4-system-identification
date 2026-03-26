#pragma once

#include <gz/msgs/actuators.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace px4::simulation
{
class SystemIdentificationLoggerPrivate;

class SystemIdentificationLogger
	: public gz::sim::System,
	  public gz::sim::ISystemConfigure,
	  public gz::sim::ISystemPreUpdate
{
public:
	SystemIdentificationLogger();
	~SystemIdentificationLogger() override;

	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) override;

private:
	std::unique_ptr<SystemIdentificationLoggerPrivate> dataPtr;
};
} // namespace px4::simulation
