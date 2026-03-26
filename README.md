PX4 System Identification Workspace
==================================

This directory is a standalone extraction of the PX4/Gazebo system-identification work from the larger `px4-custom` project.

What is included
- `experimental_validation/`
  Estimation, comparison, composite candidate builder, and tests.
- `tools/optimization/`
  The execution-side scripts needed to run identification plans.
- `overlay/`
  The PX4-side files that must be copied into a PX4 workspace to enable the identification missions and Gazebo truth logging.
- `examples/`
  Example identification plans.
- `system_identification.txt`
  Detailed method description for paper drafting.

Intended use
1. Clone PX4 separately.
2. Copy the files from `overlay/` into the PX4 tree.
3. Use the scripts under `tools/optimization/` and `experimental_validation/` to run Gazebo identification and build a digital-twin candidate.

This extracted workspace does not contain the full PX4 repository.
