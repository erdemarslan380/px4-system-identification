from __future__ import annotations

import inspect
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

import experimental_validation.run_sitl_validation as sitl_validation


class RunSitlValidationFlowTests(unittest.TestCase):
    def test_append_path_list_preserves_priority_order(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            override = root / "override_models"
            models = root / "models"
            worlds = root / "worlds"
            existing = root / "existing"
            result = sitl_validation._append_path_list(str(existing), override, models, worlds)
            self.assertEqual(
                result.split(":"),
                [
                    str(override.resolve()),
                    str(models.resolve()),
                    str(worlds.resolve()),
                    str(existing),
                ],
            )

    def test_validation_uses_takeoff_hover_then_offboard_handover(self) -> None:
        source = inspect.getsource(sitl_validation.run_validation_model)
        self.assertIn("_wait_for_ready_for_takeoff(session)", source)
        self.assertIn("_wait_for_preflight_ok(session)", source)
        self.assertIn("_arm_via_internal_command(session, mav, attempts=SITL_ARM_ATTEMPTS)", source)
        self.assertIn("manual_control = _ManualControlThread(mav)", source)
        self.assertIn('_wait_for_takeoff_hover(', source)
        self.assertIn("target_yaw=hover_yaw", source)
        self.assertIn('_land_in_posctl_with_manual_control(', source)
        self.assertIn("land_result = _land_in_posctl_with_manual_control(", source)
        self.assertIn('disarmed={land_result[\'disarmed\']}', source)
        self.assertIn("_enter_posctl_with_manual_control(session, mav, manual_control)", source)
        self.assertIn("_stabilize_direct_hover(", source)
        self.assertIn("SITL_ALLOW_UNSTABLE_CUSTOM_HOLD", source)
        self.assertIn("_start_direct_setpoint_stream(", source)
        self.assertIn('set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('session.send(\n                "trajectory_reader abs_ref "', source)
        self.assertIn("set_offboard(mav)", source)
        self.assertIn('set_param(mav, "TRJ_POS_ABS", 0', source)
        self.assertIn('set_param(mav, "TRJ_POS_YAW", hover_yaw', source)
        self.assertIn("wait_for_ground_quiet(", source)
        self.assertIn("set_anchor_from_position(mav, common_anchor[0], common_anchor[1], common_anchor[2])", source)
        self.assertIn('label="Custom hold"', source)
        self.assertIn("hover_yaw = _capture_locked_yaw(mav)", source)
        self.assertNotIn('session.send_no_wait("commander takeoff")', source)
        self.assertNotIn('session.send_no_wait("commander mode auto:land")', source)
        self.assertNotIn('session.expect("Ready for takeoff!"', source)
        self.assertNotIn("prepare_hover_and_anchor(", source)
        self.assertNotIn('session.send("commander takeoff", timeout_s=10.0)', source)
        self.assertLess(source.index("_wait_for_ready_for_takeoff(session)"), source.index("_arm_via_internal_command(session, mav, attempts=SITL_ARM_ATTEMPTS)"))
        self.assertLess(source.index("_wait_for_preflight_ok(session)"), source.index("_arm_via_internal_command(session, mav, attempts=SITL_ARM_ATTEMPTS)"))
        self.assertLess(source.index("_wait_for_ready_for_takeoff(session)"), source.index("_wait_for_preflight_ok(session)"))
        self.assertLess(source.index("_arm_via_internal_command(session, mav, attempts=SITL_ARM_ATTEMPTS)"), source.index("manual_control = _ManualControlThread(mav)"))
        self.assertLess(source.index("manual_control = _ManualControlThread(mav)"), source.index("_wait_for_takeoff_hover("))
        self.assertLess(source.index("_wait_for_takeoff_hover("), source.index("_start_direct_setpoint_stream("))
        self.assertLess(source.index("_start_direct_setpoint_stream("), source.index("set_offboard(mav)"))
        self.assertLess(source.index("set_offboard(mav)"), source.index("_stabilize_direct_hover("))
        self.assertLess(source.index("_stabilize_direct_hover("), source.rindex('set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)'))
        self.assertLess(source.index("_stabilize_direct_hover("), source.rindex('set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)'))
        self.assertLess(source.index("_stabilize_direct_hover("), source.index('session.send("custom_pos_control start")'))
        self.assertLess(source.rindex('set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)'), source.index('session.send("custom_pos_control start")'))
        self.assertLess(source.rindex('set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)'), source.index('session.send("custom_pos_control start")'))
        self.assertLess(source.index('session.send("custom_pos_control start")'), source.index('session.send("trajectory_reader start")'))
        self.assertLess(source.index("_wait_for_takeoff_hover("), source.rindex("_enter_posctl_with_manual_control(session, mav, manual_control)"))

    def test_validation_starts_built_in_trajectory_via_params(self) -> None:
        source = inspect.getsource(sitl_validation.run_validation_model)
        helper_source = inspect.getsource(sitl_validation._arm_via_internal_command)
        preflight_source = inspect.getsource(sitl_validation._wait_for_preflight_ok)
        self.assertIn('set_param(mav, "COM_RC_IN_MODE", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', inspect.getsource(sitl_validation._enter_posctl_with_manual_control))
        self.assertIn('set_param(mav, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "CBRK_SUPPLY_CHK", 894281, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "COM_DISARM_PRFLT", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)', source)
        self.assertIn('set_param(mav, "MAV_0_BROADCAST", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)', source)
        self.assertIn('set_param(mav, "MIS_TAKEOFF_ALT", abs(SITL_HOVER_Z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)', source)
        self.assertIn("if not headless and SITL_QGC_DISCOVERY_GRACE_SECONDS > 0.0:", source)
        self.assertIn("time.sleep(SITL_PARAM_PROPAGATION_SECONDS)", source)
        self.assertIn('session.send_no_wait("commander arm")', helper_source)
        self.assertIn('session.sync_prompt(timeout_s=1.0)', helper_source)
        self.assertIn("mav.motors_armed()", helper_source)
        self.assertIn('session.send("listener health_report 1", timeout_s=10.0)', helper_source)
        self.assertIn('session.send("listener failsafe_flags 1", timeout_s=10.0)', helper_source)
        self.assertIn('session.send("commander check", timeout_s=10.0)', preflight_source)
        self.assertIn('"Preflight check: OK"', preflight_source)
        self.assertNotIn('"Ready for takeoff!" in response', preflight_source)
        self.assertIn('set_anchor_from_position(mav, common_anchor[0], common_anchor[1], common_anchor[2])', source)
        self.assertIn('set_param(mav, "TRJ_ACTIVE_ID", entry.traj_id', source)
        self.assertIn('set_param(mav, "TRJ_MODE_CMD", 1', source)
        self.assertIn("entry.nominal_duration_s", source)
        self.assertIn("_install_validation_trajectories_for_locked_yaw(", source)
        self.assertIn("locked_yaw_rad=hover_yaw", source)
        self.assertLess(
            source.index("hover_yaw = _capture_locked_yaw(mav)"),
            source.index("_install_validation_trajectories_for_locked_yaw("),
        )
        self.assertLess(
            source.index('copied_log = _copy_log(tracking_log, result_root / "tracking_logs" / f"{entry.name}.csv")'),
            source.index("tracking_metrics = _tracking_log_error_metrics(tracking_log)"),
        )
        self.assertNotIn('session.send_no_wait("commander mode offboard")', source)

    def test_yaw_lock_helpers_exist_for_manual_takeoff_and_landing(self) -> None:
        takeoff_source = inspect.getsource(sitl_validation._wait_for_takeoff_hover)
        landing_source = inspect.getsource(sitl_validation._land_in_posctl_with_manual_control)
        rudder_source = inspect.getsource(sitl_validation._manual_rudder_for_yaw)
        capture_yaw_source = inspect.getsource(sitl_validation._capture_locked_yaw)
        export_source = inspect.getsource(sitl_validation._install_validation_trajectories_for_locked_yaw)

        self.assertIn("target_yaw: float | None = None", takeoff_source)
        self.assertIn("_manual_rudder_for_yaw", takeoff_source)
        self.assertIn("yaw=", takeoff_source)
        self.assertIn("_landing_throttle_for_altitude", landing_source)
        self.assertIn("rudder_cmd = 0", landing_source)
        self.assertIn("Touchdown detected; waiting for auto-disarm", landing_source)
        self.assertIn("if not mav.motors_armed()", landing_source)
        self.assertIn('"disarmed": True', landing_source)
        self.assertIn("_wrap_pi", rudder_source)
        self.assertIn("_sample_attitude", capture_yaw_source)
        self.assertIn("freeze_yaw=SITL_FREEZE_TRAJECTORY_YAW", export_source)
        self.assertIn("yaw_value=float(locked_yaw_rad)", export_source)
        self.assertEqual(sitl_validation._manual_rudder_for_yaw(0.0, 0.0), 0)
        self.assertGreater(sitl_validation._manual_rudder_for_yaw(0.0, 0.4), 0)
        self.assertLess(sitl_validation._manual_rudder_for_yaw(0.4, 0.0), 0)

    def test_validation_uses_session_start_retry_and_optional_esc_override(self) -> None:
        model_source = inspect.getsource(sitl_validation.run_validation_model)
        main_source = inspect.getsource(sitl_validation.main)
        retry_source = inspect.getsource(sitl_validation._start_session_with_retries)
        cleanup_source = inspect.getsource(sitl_validation._cleanup_background_services)
        start_source = inspect.getsource(sitl_validation.Px4SitlSession.start)
        heartbeat_source = inspect.getsource(sitl_validation.Px4SitlSession._wait_for_vehicle_heartbeat)

        self.assertIn("_start_session_with_retries(", model_source)
        self.assertIn("_cleanup_background_services()", model_source)
        self.assertIn('if model_spec.label != "stock_sitl_placeholder":', model_source)
        self.assertIn("_prepare_model_override(px4_root, model_spec.gz_model, override_models_root)", model_source)
        self.assertIn("_patch_gz_env_model_override(run_rootfs, override_models_root)", model_source)
        self.assertIn("_prepare_reference_world(px4_root, runtime_root)", model_source)
        self.assertIn("_patch_gz_env_world_override(run_rootfs, override_worlds_root)", model_source)
        self.assertIn('if sitl_esc_max is not None:', model_source)
        self.assertIn('_apply_x500_esc_scaling(mav, min_value=sitl_esc_min, max_value=sitl_esc_max)', model_source)
        self.assertIn('if sitl_hover_thrust is not None:', model_source)
        self.assertIn('set_param(mav, "MPC_THR_HOVER", sitl_hover_thrust', model_source)
        self.assertIn('_estimate_candidate_hover_thrust(candidate_dir, esc_min=sitl_esc_min, esc_max=sitl_esc_max)', model_source)
        self.assertIn('set_param(mav, "MPC_THR_HOVER", hover_thrust', model_source)
        self.assertIn("CLEANUP_SCRIPT.exists()", cleanup_source)
        self.assertIn("subprocess.run(", cleanup_source)
        self.assertIn("XEPHYR_PATTERN", cleanup_source)
        self.assertIn('time.sleep(SITL_START_RETRY_DELAY_SECONDS)', retry_source)
        self.assertIn("self._wait_for_vehicle_heartbeat(timeout_s=15.0)", start_source)
        self.assertIn('recv_match(type="HEARTBEAT"', heartbeat_source)
        self.assertIn('--sitl-esc-max', main_source)
        self.assertIn('--sitl-esc-min', main_source)
        self.assertIn('--sitl-hover-thrust', main_source)
        self.assertIn("except KeyboardInterrupt:", main_source)
        self.assertIn('_cleanup_background_services()', main_source)

    def test_main_cleans_background_services_on_keyboard_interrupt(self) -> None:
        with mock.patch.object(sitl_validation.argparse.ArgumentParser, "parse_args") as parse_args_mock:
            parse_args_mock.return_value = mock.Mock(
                px4_root="~/PX4-Autopilot-Identification",
                out_root="/tmp/sitl-test",
                candidate_dir="/tmp/candidate",
                trajectory_names="",
                model_labels="",
                sitl_esc_min=sitl_validation.SITL_ESC_MIN_DEFAULT,
                sitl_esc_max=None,
                sitl_hover_thrust=None,
                visual=False,
                show_console=False,
            )
            with mock.patch.object(
                sitl_validation,
                "run_validation_suite",
                side_effect=KeyboardInterrupt(),
            ):
                with mock.patch.object(sitl_validation, "_cleanup_background_services") as cleanup_mock:
                    self.assertEqual(sitl_validation.main(), 130)

        cleanup_mock.assert_called_once_with()

    def test_visual_mode_can_show_console_and_fix_topdown_camera(self) -> None:
        env_source = inspect.getsource(sitl_validation._build_px4_env)
        model_source = inspect.getsource(sitl_validation.run_validation_model)
        helper_source = inspect.getsource(sitl_validation._open_console_window)
        focus_source = inspect.getsource(sitl_validation._focus_window_by_title)
        restore_source = inspect.getsource(sitl_validation._restore_window_by_title)
        promote_source = inspect.getsource(sitl_validation._promote_window)
        nested_source = inspect.getsource(sitl_validation._start_nested_visual_display)
        main_source = inspect.getsource(sitl_validation.main)

        self.assertIn('env["GZ_IP"] = "127.0.0.1"', env_source)
        self.assertNotIn('env["PX4_SIM_SPEED_FACTOR"]', env_source)
        self.assertIn('env.setdefault("PX4_GZ_NO_FOLLOW", "1")', env_source)
        self.assertIn("SITL_VISUAL_DISABLE_FOLLOW = True", inspect.getsource(sitl_validation))
        self.assertIn("VISUAL_TRACK_PGAIN = 0.0", inspect.getsource(sitl_validation))
        self.assertIn("_configure_visual_camera_follow(model_name=model_spec.gz_model, env=env)", model_source)
        camera_source = inspect.getsource(sitl_validation._configure_visual_camera_follow)
        self.assertIn('env.get("PX4_GZ_NO_FOLLOW")', camera_source)
        self.assertIn('"/gui/track"', camera_source)
        self.assertIn('track_pgain: {track_pgain}', camera_source)
        self.assertIn("_open_console_window(run_rootfs / \"px4_console.log\")", model_source)
        self.assertIn("display_session = _start_nested_visual_display(runtime_root)", model_source)
        self.assertIn('env["DISPLAY"] = display_session.display', model_source)
        self.assertIn("_promote_window(visual_window_title)", model_source)
        self.assertIn("if show_console and not headless:", model_source)
        self.assertIn("Xephyr", nested_source)
        self.assertIn('"-title"', nested_source)
        self.assertIn("SITL_NESTED_DISPLAY_TITLE", nested_source)
        self.assertIn("_wait_for_window_by_title(SITL_NESTED_DISPLAY_TITLE, timeout_s=5.0)", nested_source)
        self.assertIn("gnome-terminal", helper_source)
        self.assertIn('--geometry={SITL_CONSOLE_COLUMNS}x{SITL_CONSOLE_ROWS}+0+0', helper_source)
        self.assertNotIn("--maximize", helper_source)
        self.assertIn('["wmctrl", "-a", title]', focus_source)
        self.assertIn('["wmctrl", "-R", title]', focus_source)
        self.assertIn('["wmctrl", "-r", title, "-t", str(desktop)]', restore_source)
        self.assertIn('["xdotool", "search", "--name", title]', restore_source)
        self.assertIn('["xdotool", "windowactivate", "--sync", window_id]', restore_source)
        self.assertIn("_restore_window_by_title(title)", promote_source)
        self.assertIn("_set_window_sticky(title, enabled=True)", promote_source)
        self.assertIn("_set_window_above(title, enabled=True)", promote_source)
        self.assertIn("_focus_window_by_title(title)", promote_source)
        self.assertIn("wmctrl", helper_source)
        self.assertIn("--show-console", main_source)

    def test_prepare_reference_world_adds_red_marker_next_to_takeoff(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            worlds = root / "Tools" / "simulation" / "gz" / "worlds"
            worlds.mkdir(parents=True)
            (worlds / "default.sdf").write_text(
                '<sdf version="1.9"><world name="default"><physics type="ode"/></world></sdf>',
                encoding="utf-8",
            )

            world_name, override_worlds_root, world_sdf = sitl_validation._prepare_reference_world(root, root / "runtime")

            self.assertEqual(world_name, sitl_validation.SITL_REFERENCE_WORLD_NAME)
            self.assertEqual(world_sdf.parent, override_worlds_root)
            tree = ET.parse(world_sdf)
            marker = tree.find(f".//model[@name='{sitl_validation.SITL_REFERENCE_MARKER_NAME}']")
            self.assertIsNotNone(marker)
            self.assertEqual(
                marker.findtext("pose"),
                (
                    f"{sitl_validation.SITL_REFERENCE_MARKER_X:.3f} "
                    f"{sitl_validation.SITL_REFERENCE_MARKER_Y:.3f} "
                    f"{sitl_validation.SITL_REFERENCE_MARKER_Z:.3f} 0 0 0"
                ),
            )
            self.assertEqual(marker.findtext(".//visual/material/diffuse"), "0.90 0.10 0.10 1")

    def test_patch_gz_env_world_override_rewrites_px4_world_path(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            run_rootfs = Path(td) / "rootfs"
            run_rootfs.mkdir(parents=True)
            gz_env = run_rootfs / "gz_env.sh"
            gz_env.write_text(
                "export PX4_GZ_MODELS=/tmp/models\n"
                "export PX4_GZ_WORLDS=/tmp/worlds\n"
                "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS\n",
                encoding="utf-8",
            )
            override_worlds_root = Path(td) / "override_worlds"
            override_worlds_root.mkdir()

            sitl_validation._patch_gz_env_world_override(run_rootfs, override_worlds_root)

            text = gz_env.read_text(encoding="utf-8")
            self.assertIn(f"export PX4_GZ_WORLDS={override_worlds_root.resolve()}", text)
            self.assertNotIn("export PX4_GZ_WORLDS=/tmp/worlds", text)

    def test_patch_gz_env_model_override_rewrites_px4_model_path(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            run_rootfs = Path(td) / "rootfs"
            run_rootfs.mkdir(parents=True)
            gz_env = run_rootfs / "gz_env.sh"
            gz_env.write_text(
                "export PX4_GZ_MODELS=/tmp/models\n"
                "export PX4_GZ_WORLDS=/tmp/worlds\n"
                "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS\n",
                encoding="utf-8",
            )
            override_models_root = Path(td) / "override_models"
            override_models_root.mkdir()

            sitl_validation._patch_gz_env_model_override(run_rootfs, override_models_root)

            text = gz_env.read_text(encoding="utf-8")
            self.assertIn(f"export PX4_GZ_MODELS={override_models_root.resolve()}", text)
            self.assertNotIn("export PX4_GZ_MODELS=/tmp/models", text)

    def test_patch_run_rootfs_gcs_link_enables_broadcast_on_gcs_port(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            run_rootfs = Path(td) / "rootfs"
            init_dir = run_rootfs / "etc" / "init.d-posix"
            init_dir.mkdir(parents=True)
            rc_mavlink = init_dir / "px4-rc.mavlink"
            rc_mavlink.write_text(
                "#!/bin/sh\n"
                "mavlink start -x -u $udp_gcs_port_local -r 4000000 -f\n"
                "mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote\n",
                encoding="utf-8",
            )

            sitl_validation._patch_run_rootfs_gcs_link(run_rootfs)

            text = rc_mavlink.read_text(encoding="utf-8")
            self.assertIn("mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -p", text)
            self.assertIn(
                "mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote",
                text,
            )

    def test_session_send_waits_for_next_prompt_not_current_prompt_echo(self) -> None:
        send_source = inspect.getsource(sitl_validation.Px4SitlSession.send)
        sync_source = inspect.getsource(sitl_validation.Px4SitlSession.sync_prompt)
        shutdown_source = inspect.getsource(sitl_validation.Px4SitlSession.shutdown)

        self.assertIn("PX4_NEXT_PROMPT_RE", send_source)
        self.assertNotIn('expect_exact(PX4_PROMPT', send_source)
        self.assertIn("PX4_NEXT_PROMPT_RE", sync_source)
        self.assertIn('self.send_no_wait("shutdown")', shutdown_source)
        self.assertIn("self._child.expect(pexpect.EOF, timeout=3.0)", shutdown_source)
        self.assertNotIn('self.send("shutdown", timeout_s=10)', shutdown_source)
        self.assertIsNotNone(sitl_validation.PX4_NEXT_PROMPT_RE.search("\rpxh> "))
        self.assertIsNotNone(sitl_validation.PX4_NEXT_PROMPT_RE.search("\r\x1b[0mpxh> "))
        self.assertIsNone(sitl_validation.PX4_NEXT_PROMPT_RE.search("\rpxh> commander arm"))

    def test_wait_for_ready_for_takeoff_returns_immediately_if_console_already_contains_ready_line(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            console_log = Path(td) / "px4_console.log"
            console_log.write_text("INFO  [commander] Ready for takeoff!\n", encoding="utf-8")

            class FakeSession:
                def __init__(self, path: Path) -> None:
                    self._console_log = path
                    self.expect = mock.Mock()

            session = FakeSession(console_log)
            sitl_validation._wait_for_ready_for_takeoff(session)
            session.expect.assert_not_called()

    def test_restore_window_by_title_uses_current_workspace_and_activation(self) -> None:
        with mock.patch.object(sitl_validation.shutil, "which", side_effect=lambda name: "/usr/bin/" + name):
            with mock.patch.object(sitl_validation, "_current_desktop_index", return_value=1):
                completed = mock.Mock(stdout="100\n")
                with mock.patch.object(sitl_validation.subprocess, "run", return_value=completed) as run_mock:
                    sitl_validation._restore_window_by_title("Gazebo Sim")

        calls = [call.args[0] for call in run_mock.call_args_list]
        self.assertIn(["wmctrl", "-r", "Gazebo Sim", "-t", "1"], calls)
        self.assertIn(["wmctrl", "-r", "Gazebo Sim", "-b", "remove,hidden,shaded,skip_taskbar,skip_pager"], calls)
        self.assertIn(["xdotool", "search", "--name", "Gazebo Sim"], calls)
        self.assertIn(["xdotool", "windowmap", "100"], calls)
        self.assertIn(["xdotool", "windowraise", "100"], calls)
        self.assertIn(["xdotool", "windowactivate", "--sync", "100"], calls)

    def test_window_exists_by_title_checks_wmctrl_then_xdotool(self) -> None:
        wmctrl_result = mock.Mock(stdout="0x1 0 host PX4 Gazebo Nested Display\n", returncode=0)
        with mock.patch.object(sitl_validation.shutil, "which", side_effect=lambda name: "/usr/bin/" + name):
            with mock.patch.object(sitl_validation.subprocess, "run", return_value=wmctrl_result):
                self.assertTrue(sitl_validation._window_exists_by_title("PX4 Gazebo Nested Display"))

        def _which(name: str) -> str | None:
            return None if name == "wmctrl" else f"/usr/bin/{name}"

        with mock.patch.object(sitl_validation.shutil, "which", side_effect=_which):
            with mock.patch.object(sitl_validation.subprocess, "run", return_value=mock.Mock(returncode=0)):
                self.assertTrue(sitl_validation._window_exists_by_title("PX4 Gazebo Nested Display"))

    def test_find_free_x_display_number_skips_used_sockets_and_locks(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            x11_dir = root / ".X11-unix"
            x11_dir.mkdir()
            (x11_dir / "X90").touch()
            (root / ".X91-lock").touch()
            with mock.patch.object(sitl_validation, "Path", side_effect=lambda value: x11_dir if value == "/tmp/.X11-unix" else root if value == "/tmp" else Path(value)):
                self.assertEqual(sitl_validation._find_free_x_display_number(), 92)

    def test_estimate_candidate_hover_thrust_uses_candidate_mass_and_motor_model(self) -> None:
        candidate_dir = Path("examples/paper_assets/candidates/jmavsim_prior_v1")
        hover = sitl_validation._estimate_candidate_hover_thrust(candidate_dir, esc_min=0, esc_max=1000)
        self.assertIsNotNone(hover)
        assert hover is not None
        self.assertAlmostEqual(hover, 0.7002374597, places=6)

    def test_manual_takeoff_throttle_schedule_is_clamped_and_increasing_with_error(self) -> None:
        far = sitl_validation._throttle_for_target(0.0, -5.0)
        near = sitl_validation._throttle_for_target(-4.7, -5.0)
        overshoot = sitl_validation._throttle_for_target(-5.4, -5.0)
        deep_overshoot = sitl_validation._throttle_for_target(-8.5, -5.0)

        self.assertGreaterEqual(far, near)
        self.assertGreaterEqual(near, overshoot)
        self.assertLessEqual(far, sitl_validation.SITL_MANUAL_MAX_THROTTLE)
        self.assertGreaterEqual(overshoot, 0)
        self.assertGreaterEqual(overshoot, deep_overshoot)
        self.assertEqual(deep_overshoot, 0)

    def test_landing_throttle_schedule_descends_more_assertively_from_altitude(self) -> None:
        self.assertEqual(sitl_validation._landing_throttle_for_altitude(-5.0, 0.0), 320)
        self.assertEqual(sitl_validation._landing_throttle_for_altitude(-2.0, 0.0), 350)
        self.assertEqual(sitl_validation._landing_throttle_for_altitude(-1.0, 0.0), 390)
        self.assertEqual(sitl_validation._landing_throttle_for_altitude(-0.4, 0.0), 435)
        self.assertEqual(sitl_validation._landing_throttle_for_altitude(-0.1, 0.0), 465)


if __name__ == "__main__":
    unittest.main()
