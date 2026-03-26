#!/usr/bin/env python3
"""Browser smoke test for the planner UI."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
import urllib.error
from pathlib import Path


def wait_for_url(url: str, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while True:
        try:
            with urllib.request.urlopen(url, timeout=1.0) as response:
                if response.status == 200:
                    return True
        except Exception:
            pass
        if time.time() > deadline:
            return False
        time.sleep(0.25)


def main() -> int:
    try:
        from selenium import webdriver
        from selenium.webdriver.common.by import By
        from selenium.webdriver.firefox.options import Options
        from selenium.webdriver.support import expected_conditions as EC
        from selenium.webdriver.support.ui import Select, WebDriverWait
    except Exception as exc:  # pragma: no cover - smoke helper
        print(json.dumps({"ok": False, "error": f"selenium import failed: {exc}"}))
        return 2

    ap = argparse.ArgumentParser(description="Planner browser smoke test.")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--server-root", default="Tools/optimization")
    ap.add_argument("--planner-path-fragment", default="example_plan.yaml")
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    server_root = (repo_root / args.server_root).resolve()
    planner_url = f"http://127.0.0.1:{args.port}/planner?fresh={int(time.time())}"

    server_proc = None
    firefox = os.environ.get("FIREFOX_BIN", "/snap/firefox/current/usr/lib/firefox/firefox")
    geckodriver = os.environ.get("GECKODRIVER_BIN", "/snap/firefox/current/usr/lib/firefox/geckodriver")
    if not Path(firefox).exists() or not Path(geckodriver).exists():
        print(json.dumps({"ok": False, "error": "firefox or geckodriver not found"}))
        return 2

    try:
        if not wait_for_url(planner_url, 1.0):
            server_proc = subprocess.Popen(
                [sys.executable, str(server_root / "serve_dashboard.py"), "--port", str(args.port), "--root", str(server_root)],
                cwd=str(repo_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            if not wait_for_url(planner_url, 30.0):
                raise RuntimeError("planner server did not become ready")

        options = Options()
        options.binary_location = firefox
        if args.headless:
            options.add_argument("--headless")
        service = webdriver.FirefoxService(executable_path=geckodriver)
        driver = webdriver.Firefox(service=service, options=options)
        wait = WebDriverWait(driver, 30)
        try:
            driver.get(planner_url)
            wait.until(lambda d: d.execute_script("return document.readyState") == "complete")
            wait.until(lambda d: d.execute_script("return !!window.Plotly"))
            wait.until(lambda d: len(Select(d.find_element(By.ID, "existing_plan_select")).options) > 1)
            select = Select(driver.find_element(By.ID, "existing_plan_select"))
            target_value = None
            for option in select.options:
                if args.planner_path_fragment in option.text:
                    target_value = option.get_attribute("value")
                    break
            if not target_value:
                raise RuntimeError(f"could not find YAML containing {args.planner_path_fragment}")
            select.select_by_value(target_value)
            wait.until(lambda d: d.find_element(By.ID, "plan_name").get_attribute("value").strip() != "")
            wait.until(lambda d: d.find_element(By.ID, "stat_tasks").text.strip() not in {"", "0"})
            wait.until(lambda d: d.execute_script("var gd=document.getElementById('trajectory_plot'); return gd && gd.data && gd.data.length >= 5;"))
            wait.until(lambda d: d.find_element(By.ID, "task_mission_mode"))
            wait.until(lambda d: d.find_element(By.ID, "task_base_param_file"))

            driver.execute_script(
                "var gd=document.getElementById('trajectory_plot'); Plotly.relayout(gd, {'scene.camera.eye.x': 2.4, 'scene.camera.eye.y': 0.8, 'scene.camera.eye.z': 0.6});"
            )
            time.sleep(0.4)
            before = driver.execute_script("var gd=document.getElementById('trajectory_plot'); return gd.layout.scene.camera.eye;")
            slider = driver.find_element(By.ID, "trajectory_slider")
            max_value = int(slider.get_attribute("max") or "0")
            driver.execute_script(
                "arguments[0].value = arguments[1]; arguments[0].dispatchEvent(new Event('input', {bubbles:true}));",
                slider,
                str(max(0, max_value // 2)),
            )
            time.sleep(0.8)
            after = driver.execute_script("var gd=document.getElementById('trajectory_plot'); return gd.layout.scene.camera.eye;")
            tooltip = driver.execute_script(
                "var el=document.querySelector('#param_grid_body .param-help'); return el ? el.getAttribute('title') : '';"
            )
            mission_mode = Select(driver.find_element(By.ID, "task_mission_mode")).first_selected_option.get_attribute("value")

            payload = {
                "ok": True,
                "plan_name": driver.find_element(By.ID, "plan_name").get_attribute("value"),
                "mission_mode": mission_mode,
                "task_evals": driver.find_element(By.ID, "stat_task_evals").text,
                "task_eta": driver.find_element(By.ID, "stat_task_eta").text,
                "plan_tasks": driver.find_element(By.ID, "stat_tasks").text,
                "plan_evals": driver.find_element(By.ID, "stat_evals").text,
                "plan_eta": driver.find_element(By.ID, "stat_eta").text,
                "avg_eval": driver.find_element(By.ID, "stat_eval").text,
                "tooltip": tooltip,
                "camera_before": before,
                "camera_after": after,
                "trajectory_status": driver.find_element(By.ID, "trajectory_status").text,
            }
            print(json.dumps(payload, indent=2))
            return 0
        finally:
            driver.quit()
    finally:
        if server_proc is not None and server_proc.poll() is None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
