#!/usr/bin/env bash
# -----------------------------------------------------------------------
# Gazebo Environment Configuration
# -----------------------------------------------------------------------
# GZ_SIM_RESOURCE_PATH: Where Gazebo looks for models and worlds
# GZ_SIM_SYSTEM_PLUGIN_PATH: Where Gazebo looks for plugin libraries
# GZ_SIM_SERVER_CONFIG_PATH: Custom Gazebo server configuration file
#
# See Gazebo docs
# https://gazebosim.org/api/sim/8/resources.html
# https://gazebosim.org/api/sim/8/server_config.html
# -----------------------------------------------------------------------

export PX4_GZ_MODELS=/home/earsub/px4-system-identification/sitl_runs/hitl_bridge_major_param_hairpin_20260411/sitl_candidate_repeatability/run_05/cases/hairpin/runtime/digital_twin/override_models
export PX4_GZ_WORLDS=/home/earsub/px4-system-identification/sitl_runs/hitl_bridge_major_param_hairpin_20260411/sitl_candidate_repeatability/run_05/cases/hairpin/runtime/digital_twin/override_worlds
export PX4_GZ_PLUGINS=/home/earsub/PX4-Autopilot-Identification/build/px4_sitl_default/src/modules/simulation/gz_plugins
export PX4_GZ_SERVER_CONFIG=/home/earsub/PX4-Autopilot-Identification/src/modules/simulation/gz_bridge/server.config

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=$PX4_GZ_SERVER_CONFIG
