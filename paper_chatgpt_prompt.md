# Paper Workflow Prompt

Use the block below as a ready-to-paste prompt for ChatGPT when drafting a paper, report section, thesis subsection, or structured technical summary about this repository.

## Ready-to-Paste Prompt

```text
You are helping write an academic paper about a PX4-based multicopter system-identification workflow. Write in clear technical English unless I explicitly ask for another language. Use only the facts below. Do not invent results, measurements, or completion claims that are not explicitly marked as verified.

Repository and scope
- The project is a standalone PX4 system-identification workspace.
- The purpose is to:
  1. run identification maneuvers,
  2. estimate an x500-compatible SDF digital-twin candidate,
  3. validate that candidate on five fixed trajectories,
  4. compare reference, stock SITL, identified/tuned SITL, and real-flight traces in a common visualization format.
- The repository intentionally separates this identification workflow from any older optimization or planner stack.

Main workflow stages
1. Overlay and workspace preparation
   - A custom overlay is synchronized into a dedicated PX4 workspace.
   - The dedicated PX4 tree is `~/PX4-Autopilot-Identification`.
   - The overlay adds:
     - `custom_pos_control`
     - `trajectory_reader`
     - a Gazebo truth logger for SITL
   - The repository side contains estimation, comparison, validation, and figure-generation scripts under `experimental_validation/`.

2. Fixed identification maneuvers
   - The workflow uses nine fixed maneuvers:
     - `hover_thrust`
     - `mass_vertical`
     - `roll_sweep`
     - `pitch_sweep`
     - `yaw_sweep`
     - `drag_x`
     - `drag_y`
     - `drag_z`
     - `motor_step`
   - These maneuvers can be run:
     - one by one,
     - as `identification_only`,
     - or as part of `full_stack`.

3. Fixed validation trajectories
   - The workflow uses five fixed trajectory files only. No new validation trajectories are synthesized.
   - IDs and meanings:
     - `100`: hairpin, about 23 s
     - `101`: lemniscate, about 19 s
     - `102`: circle, about 15 s
     - `103`: time-optimal path, about 11 s
     - `104`: minimum-snap path, about 14 s
   - These can be run:
     - one by one,
     - as `trajectory_only`,
     - or as part of `full_stack`.

4. Campaign structure
   - The same campaign names are reused across SITL, HIL, and real flight:
     - `identification_only`
     - `trajectory_only`
     - `full_stack`
   - `full_stack` means:
     - first the 9 identification maneuvers,
     - then the 5 validation trajectories.
   - Return-to-anchor segments are intentionally not written into the CSV logs.

5. SITL methodology
   - SITL runs on Gazebo, not jMAVSim.
   - SITL is used for:
     - baseline system-identification validation,
     - off-nominal robustness checks,
     - identified-candidate validation,
     - comparison-figure generation.
   - In SITL, Gazebo truth logs are available, so this is the strongest current validation mode.
   - The methodology includes:
     - running identification maneuvers,
     - estimating parameters from logs,
     - preparing an identified model,
     - rerunning the five validation trajectories,
     - comparing stock SITL and identified/tuned SITL against reference.

6. Off-nominal SITL stress check
   - A separate off-nominal SITL study exists.
   - It perturbs the x500 model and adds light wind.
   - The point is to verify that:
     - the workflow can still identify a changed model,
     - the resulting trajectories visibly deviate from stock behavior,
     - the figure pipeline can compare multiple datasets in a consistent format.

7. HIL purpose and limitations
   - HIL uses CubeOrange and jMAVSim.
   - HIL is not treated as a second full scientific validation environment.
   - The HIL goal is narrower:
     - verify that the real board can execute one uninterrupted campaign,
     - verify that CSV logs are written to the SD card,
     - verify that USB CDC / MAVFTP review flow works,
     - verify whether RAM and CPU stay within acceptable limits.
   - HIL should therefore be described as a smoke-test or pre-flight robustness stage, not as the main source of model-identification truth.

8. HIL resource reporting
   - HIL resource reporting is file-based, not anecdotal.
   - After HIL, the PX4 `.ulg` log is pulled from the SD card and summarized with `experimental_validation/report_hil_resources.py`.
   - This script reads the PX4 `cpuload` topic and outputs a JSON summary for:
     - board CPU load,
     - board RAM usage,
     - issue counts such as:
       - `RAM usage too high`
       - `low on stack`
       - `parameters verify: failed`
       - `BARO #0 failed`
       - `failsafe activated`
   - Current verified repository state:
     - one short HIL ULog snapshot was healthy at about `27.6%` CPU and `28.3%` RAM,
     - one longer HIL ULog snapshot failed with:
       - `RAM usage too high: 96657.7%`
       - `wq:nav_and_controllers low on stack! (0 bytes left)`
       - `parameters verify: failed (-1)`
     - therefore HIL is currently not signed off as fully healthy.
   - If you describe HIL status, keep that distinction honest.

9. RC-assisted operation
   - The repository supports RC-driven selection for controller, workflow, and item selection.
   - It also supports an `H` trigger button that applies the currently selected action.
   - This is intended for HIL and real-flight operator convenience.
   - The PX4 flight-mode switch remains separate from the repository-specific workflow selection.
   - A physical receiver may stay connected during HIL while jMAVSim is running because the simulator uses the USB CDC MAVLink path, not the board’s RC input wiring.

10. Real-flight workflow
   - Real flight reuses the same campaign logic:
     - one maneuver,
     - one trajectory,
     - `identification_only`,
     - `trajectory_only`,
     - `full_stack`
   - The intended real-flight operator flow is:
     - take off manually,
     - stabilize near the anchor,
     - enter `OFFBOARD`,
     - run the selected campaign,
     - land after the last item.
   - The goal is to collect:
     - identification CSV logs,
     - tracking CSV logs,
     - and optionally `.ulg` logs for board-health review.

11. Log movement and review
   - Logs can be pulled without removing the SD card by using USB CDC plus MAVFTP.
   - The repository includes both:
     - a MAVFTP pull script,
     - and a browser-style SD-card inspection tool.
   - After a HIL or real-flight session:
     - raw logs are copied into a repository session folder,
     - an interactive review bundle can be generated,
     - and the same logs can feed identification and follow-up SITL validation.

12. Identification-to-validation loop
   - The intended loop is:
     1. collect identification logs,
     2. estimate candidate parameters,
     3. build an identified x500 model,
     4. run five validation trajectories in SITL,
     5. compare stock SITL, identified/tuned SITL, and flight data.

13. Figure policy
   - All main comparison figures should use the same visual structure.
   - The preferred four-layer trajectory comparison format is:
     - dashed `Reference`
     - `Stock x500 SITL`
     - `Identified/Tuned SITL`
     - `Real flight results`
   - The same grouped-figure format is used for:
     - off-nominal SITL checks,
     - future HIL-identified SITL comparisons,
     - future real-flight-identified SITL comparisons,
     - current stock-vs-real-flight comparisons.
   - Figures should appear inline in the documentation at the relevant section, not only at the end.

14. Current verified result blocks
   - The repository already includes:
     - off-nominal SITL comparison figures,
     - current imported real-flight baseline PID comparison figures,
     - placeholders for HIL-identified SITL figures,
     - placeholders for future real-flight-identified SITL figures.
   - Important honesty rule:
     - HIL-identified SITL figures should be described as placeholders until honest HIL identification CSV logs exist.
     - Real-flight-identified SITL figures should also remain placeholders until real-flight identification logs exist.

15. Truthfulness rules for writing
   - Do not say HIL is fully validated unless the HIL resource report and HIL campaign logs are clean.
   - Do not say real-flight identification is complete unless actual real-flight identification logs exist.
   - Do not claim that placeholder figures are measured results.
   - Distinguish clearly between:
     - verified SITL results,
     - current imported real-flight baseline trajectory comparisons,
     - partially verified HIL smoke-test status,
     - future or pending identified-result blocks.

16. What I want from you
   - Write a coherent paper-ready explanation of this workflow.
   - Emphasize that the methodology is staged:
     - SITL for reproducible development and upper-bound validation,
     - HIL for pre-flight board robustness and logging checks,
     - real flight for final identification data collection.
   - Present the workflow as one consistent pipeline instead of disconnected scripts.
   - When discussing results, preserve the honest status of each stage.
   - If useful, structure the output as:
     - Abstract
     - Introduction
     - System Architecture
     - Identification Maneuver Design
     - Validation Trajectory Design
     - SITL Methodology
     - HIL Smoke-Test Methodology
     - Real-Flight Data-Collection Methodology
     - Digital-Twin Estimation Pipeline
     - Results
     - Current Limitations
     - Future Work

Now produce a polished technical workflow narrative suitable for a paper or thesis section, while preserving the distinction between verified results and pending stages.
```

## Notes

- Keep this file synchronized with `README.md` and `system_identification.txt` when the workflow changes.
- If HIL becomes fully signed off later, update the HIL resource section inside the prompt before using it for a final paper draft.
