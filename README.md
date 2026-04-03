PX4 SITL Quickstart For Stock x500 Circle
=========================================

Current goal
------------
Get one clean visible `SITL` run with:
- stock `x500` Gazebo model
- one `circle` trajectory
- Gazebo GUI open
- visible PX4 console window
- fixed near-top-down camera

This README is intentionally narrow. It is the shortest path for repeating the same `circle SITL` workflow that we are debugging right now.

Workspace
---------
Use these two directories together:
- `~/PX4-Autopilot-Identification`
- `~/px4-system-identification`

If needed, sync this repo into the PX4 tree:

```bash
cd ~/px4-system-identification
./sync_into_px4_workspace.sh ~/PX4-Autopilot-Identification
```

Clean start
-----------
Before every new `SITL` attempt, close leftovers:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh
```

If you also want a best-effort USB re-enumeration for a specific serial device:

```bash
cd ~/px4-system-identification
./examples/cleanup_px4_background_services.sh --device /dev/ttyACM0 --reset-usb
```

Main command: stock x500 + circle + visual + visible console
------------------------------------------------------------
Run this from the repository root:

```bash
cd ~/px4-system-identification
python3 experimental_validation/run_sitl_validation.py \
  --trajectory-names circle \
  --model-labels stock_sitl_placeholder \
  --sitl-esc-min 0 \
  --sitl-esc-max 80 \
  --visual \
  --show-console \
  --out-root /tmp/sitl_stock_circle_visual
```

What should open
----------------
You should see two windows:
- `Gazebo`
- `PX4 SITL Log (read-only)`

The log window is a live viewer of `px4_console.log`. It is not a manual shell for typing commands.

If you need a real interactive `pxh>` shell, use the classic manual startup path:

```bash
cd ~/PX4-Autopilot-Identification
unset HEADLESS
make px4_sitl gz_x500
```

That manual path opens Gazebo and leaves the real `pxh>` shell in the same terminal.

Gazebo camera
-------------
In visual mode the runner now defaults to this fixed follow view:
- `PX4_GZ_FOLLOW_OFFSET_X=0`
- `PX4_GZ_FOLLOW_OFFSET_Y=0`
- `PX4_GZ_FOLLOW_OFFSET_Z=10`

That gives a closer top-down view than the older `z=12` setting while still keeping the vehicle and path visible.

If you want to override it for one run:

```bash
PX4_GZ_FOLLOW_OFFSET_X=0 \
PX4_GZ_FOLLOW_OFFSET_Y=0 \
PX4_GZ_FOLLOW_OFFSET_Z=12 \
python3 experimental_validation/run_sitl_validation.py \
  --trajectory-names circle \
  --model-labels stock_sitl_placeholder \
  --sitl-esc-min 0 \
  --sitl-esc-max 80 \
  --visual \
  --show-console \
  --out-root /tmp/sitl_stock_circle_visual
```

What to watch in the PX4 console
--------------------------------
During this debugging phase, these lines matter most:
- `Arming denied: ...`
- `Preflight Fail: ...`
- `Failsafe activated`
- `mode: ...`
- `OFFBOARD`

If the run fails, copy the exact `Arming denied` or `Preflight Fail` line first.

Where logs end up
-----------------
After the run, check:
- `/tmp/sitl_stock_circle_visual/runtime/stock_sitl_placeholder/rootfs/px4_console.log`
- `/tmp/sitl_stock_circle_visual/stock_sitl_placeholder/run_manifest.json`

Current debugging rule
----------------------
We are going step by step. For now do not jump to the identified model, `HIL`, or multi-trajectory runs.

Only use:
- stock `x500`
- one `circle`
- visible Gazebo
- visible PX4 console

First success criterion
-----------------------
The first target is simple:
1. PX4 arms cleanly.
2. Vehicle enters the hover/offboard handover path without immediate reject.
3. `circle` starts.

If step 1 fails, the next action is always:
- read the exact `PX4 SITL Console` arming/preflight line
- fix that one blocker
- retry the same command
