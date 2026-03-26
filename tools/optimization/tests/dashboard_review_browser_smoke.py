#!/usr/bin/env python3
"""Browser smoke test for the live monitor and results review screens."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
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
        from selenium.webdriver.support.ui import WebDriverWait
    except Exception as exc:  # pragma: no cover - smoke helper
        print(json.dumps({"ok": False, "error": f"selenium import failed: {exc}"}))
        return 2

    ap = argparse.ArgumentParser(description="Browser smoke test for live monitor and results review.")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--server-root", default="Tools/optimization")
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    server_root = (repo_root / args.server_root).resolve()
    dashboard_url = f"http://127.0.0.1:{args.port}/dashboard?fresh={int(time.time())}"
    review_url = f"http://127.0.0.1:{args.port}/review?fresh={int(time.time())}"

    server_proc = None
    firefox = os.environ.get("FIREFOX_BIN", "/snap/firefox/current/usr/lib/firefox/firefox")
    geckodriver = os.environ.get("GECKODRIVER_BIN", "/snap/firefox/current/usr/lib/firefox/geckodriver")
    if not Path(firefox).exists() or not Path(geckodriver).exists():
        print(json.dumps({"ok": False, "error": "firefox or geckodriver not found"}))
        return 2

    try:
        if not wait_for_url(dashboard_url, 1.0):
            server_proc = subprocess.Popen(
                [sys.executable, str(server_root / "serve_dashboard.py"), "--port", str(args.port), "--root", str(server_root)],
                cwd=str(repo_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            if not wait_for_url(dashboard_url, 30.0):
                raise RuntimeError("dashboard server did not become ready")

        options = Options()
        options.binary_location = firefox
        if args.headless:
            options.add_argument("--headless")
        service = webdriver.FirefoxService(executable_path=geckodriver)
        driver = webdriver.Firefox(service=service, options=options)
        wait = WebDriverWait(driver, 30)
        try:
            driver.get(dashboard_url)
            wait.until(lambda d: d.execute_script("return document.readyState") == "complete")
            wait.until(lambda d: d.find_element(By.ID, "queue_cards"))
            wait.until(lambda d: len(d.find_elements(By.CSS_SELECTOR, "#queue_cards .queue_card")) >= 1)
            wait.until(lambda d: d.find_element(By.ID, "active_task_value").text.strip() not in {"", "-"})
            wait.until(lambda d: d.find_element(By.ID, "heartbeat_value").text.strip() not in {"", "-"})
            wait.until(lambda d: d.find_element(By.ID, "task_select"))

            dashboard_payload = {
                "active_task": driver.find_element(By.ID, "active_task_value").text.strip(),
                "workers": driver.find_element(By.ID, "workers_value").text.strip(),
                "task_eta": driver.find_element(By.ID, "eta_value").text.strip(),
                "plan_eta": driver.find_element(By.ID, "plan_eta_value").text.strip(),
                "queue_cards": len(driver.find_elements(By.CSS_SELECTOR, "#queue_cards .queue_card")),
            }

            driver.get(review_url)
            wait.until(lambda d: d.execute_script("return document.readyState") == "complete")
            wait.until(lambda d: d.find_element(By.ID, "report_scope_select"))
            wait.until(lambda d: len(d.find_elements(By.CSS_SELECTOR, "#report_tables tr[data-group-id]")) >= 1)
            wait.until(lambda d: d.find_element(By.ID, "replay_mode_select"))

            report_rows = driver.find_elements(By.CSS_SELECTOR, "#report_tables tr[data-group-id]")
            driver.execute_script("arguments[0].scrollIntoView({block: 'center'});", report_rows[0])
            driver.execute_script("arguments[0].click();", report_rows[0])
            clicked_group_id = report_rows[0].get_attribute("data-group-id")
            clicked_task_id = report_rows[0].get_attribute("data-task-id")
            wait.until(lambda d: d.find_element(By.ID, "task_select").get_attribute("value").strip() == clicked_group_id)
            wait.until(lambda d: d.find_element(By.CSS_SELECTOR, "#report_tables tr.selected"))

            replay_source = driver.find_element(By.ID, "replay_source_label").get_attribute("textContent").strip()
            review_payload = {
                "report_rows": len(report_rows),
                "selected_group": driver.find_element(By.ID, "task_select").get_attribute("value"),
                "selected_task": clicked_task_id,
                "replay_source": replay_source,
                "report_scope": driver.find_element(By.ID, "report_scope_select").get_attribute("value"),
            }
            if not review_payload["replay_source"].startswith("Replay source:"):
                raise RuntimeError(f"replay source label not initialized: {review_payload}")

            print(json.dumps({
                "ok": True,
                "dashboard": dashboard_payload,
                "review": review_payload,
            }, indent=2))
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
