# EGH446 Autonomous Systems: Major Project Part B (Group 50)

Path planning and onboard obstacle avoidance for an autonomous ground robot
among unmapped obstacles.

| Name | Student no. | Contribution |
|---|---|---|
| Chabod Masere | n10695141 | Collision avoidance design: re-planning and reactive layers, sensing |
| Jonah Menzies | n11395168 | Path planning: map inflation, A* routing, PSO smoothing, waypoint ordering |
| Balin Carkeet | n11336692 | Guidance: path following, waypoint capture and control |

This folder is the unit's ground-robot simulation environment with our
estimator, guidance, path planner and collision avoidance built into it. Each
run generates a random mission: five waypoints are placed on the supplied map,
ordered into the shortest tour, joined by a smooth wall-safe path, and driven
by an Ackermann-steered robot that re-plans around each obstacle it detects.
The simulation stops by itself when the robot reaches the last waypoint.

## Full guide

**`sim_env/GettingStarted.mlx`** is the full user guide: how to run a mission,
what every line and marker on the map means, how each subsystem works, which
parameters to change, and which signals are logged. A PDF export is in
**`GettingStarted_preview.pdf`** for reading without MATLAB. This README is a
short summary of it.

## Requirements

- MATLAB **R2026a** or newer, with Simulink and Stateflow (the model is saved
  in R2026a and will not open in older releases)
- Robotics System Toolbox and Navigation Toolbox
- The Mobile Robotics Simulation Toolbox is included in `sim_env/toolboxes/MRTB`

## Quick start

1. Set MATLAB's current folder to `sim_env`.
2. Run `startupGround`. This adds the toolboxes to the path and opens the
   model `sl_groundvehicleDynamics`.
3. Press **Run** in Simulink.

The model's InitFcn loads the map (`complexMap_air_ground.mat`) and obstacles
(`obstacles_air_ground.mat`), runs `mdl_robot` to generate and plan the
mission, and runs `controller_params`. The waypoint order and path are printed
in the Command Window, each re-plan prints a line such as
`Replan 1: 1 obstacle(s) known, re-planned leg(s) 2`, and the run ends with
`Mission complete`.

Every run is a new random mission. The first run after starting MATLAB is
always the same one; uncomment `rng('shuffle')` near the top of
`toolboxes/ground_robot/mdl_robot.m` to change that.

## How it works

| Subsystem | Where | Summary |
|---|---|---|
| Mission planning | `toolboxes/ground_robot/mdl_robot.m` | Inflates the walls by 0.6 m (`inflate_map`), routes every pair of points with A* (`A_star`, `waypoint_distances`), orders the waypoints with ant colony optimisation (`wp_antColony`) and smooths each leg with particle swarm optimisation (`wp_swarm`). Uses the wall map only; obstacles are not known to the planner. |
| Re-planning | `toolboxes/ground_robot/obstacle_replan.m`, *Replanner* block | Remembers and confirms obstacles from the Object Detector, and re-plans any leg that passes within 1.0 m of one, with a 0.8 m keep-out. |
| Reactive avoidance | top-level *MATLAB Function* block (copy in `obstacle_avoidance_block.m`) | Potential field on detected obstacles and lidar wall returns; backup to re-planning. |
| Guidance | *Guidance* block (copy in `guidance_block.txt`) | Follows the active path and captures waypoints 1 to 4 within 0.4 m and the last within 0.10 m. |
| State estimation | *State Estimation and Filtering* block | Seven-state linear Kalman filter at 100 Hz. R is set from the measured sensor noise (0.10 m, 2 deg, 0.1 m/s). |
| Control | *Controller* subsystem, `controller_params.m` | Proportional heading and speed control; speed limit 1.0 m/s. |

## Main parameters

| What | File | Variable |
|---|---|---|
| Speed limit, gains, turn-rate limit | `sim_env/controller_params.m` | `v_max`, `K_theta`, `K_dist`, `omega_max` |
| Waypoint capture radius (0.4 m) | `sim_env/controller_params.m` | `capture_radius` |
| Number of waypoints, start pose | `toolboxes/ground_robot/mdl_robot.m` | `no_wps`, `robot.X`, `robot.Y`, `robot.Theta` |
| Path smoothing (PSO) | `toolboxes/ground_robot/pso_defaults.m` | `c_hard`, `c_soft`, `kappa_max`, weights |
| Ant colony settings | `toolboxes/ground_robot/aco_defaults.m` | `ACTIVE` |
| Re-planning | top of `toolboxes/ground_robot/obstacle_replan.m` | `TRIGGER`, `OBS_CLR`, `OBS_WIDE`, ... |
| Reactive avoidance | top-level *MATLAB Function* block | `d_inf`, `k_rep`, `k_tan`, `d_wall`, `k_wall` |

Code inside Simulink blocks must be edited in the block itself; the `.m` and
`.txt` copies next to the model are for reference only.

## Results (report, Section V)

Over 25 seeded missions (seeds 1 to 25), measured against the simulator's
noise-free pose:

- 25 of 25 missions completed; the waypoint order was optimal in all 25
- mean mission time 123 s for a 108 m planned path
- closest pass 0.58 m to an obstacle and 0.65 m to a wall (required 0.40 m)
- waypoints 1 to 4 passed within 0.23 m; final stop within 0.22 m

## Reproducing the report's results

The batch and analysis scripts are in `analysis/` in the submission zip
(`report/analysis` in the group repository):

- `batch_missions.m` (via `run_batch.sh`): runs seeded missions and logs the
  noise-free pose alongside the model's own signals
- `analyse_missions.m`: metrics, the speed and capture-radius sweep, and the
  report figures
- `ablation.m`: missions with the re-planner switched off
- `noise_characterise.m`, `kf_tune.m`: sensor-noise statistics and the offline
  Kalman filter tuning

`sim_env/report_plots.m` makes quick-look plots of a single run from the base
workspace.
