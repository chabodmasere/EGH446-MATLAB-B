# Report analysis (EGH446 Part B, Group 50)

Scripts that produced every number, table and figure in the report, and the
final results they produced. None of this is needed to run the simulation;
it is here so the results can be checked and repeated.

## How the results were made

1. **Run seeded missions.** `run_batch.sh OUTDIR TAG SEEDS [OVERRIDE] [SHIMDIR]`
   starts a fresh MATLAB and calls `batch_missions.m`, which runs one mission
   per seed and saves a `TAG_XX.mat` log. The report uses seeds 1 to 25
   (tag `base`). Sweeps repeat seeds 1 to 8 with one setting changed through
   `OVERRIDE`, for example `"struct('v_max',1.5)"`.
2. **Logging shims.** `logging_shims/` holds copies of `controller_params.m`
   and `path_store.m` that also log signals for analysis (and accept the sweep
   overrides). `logging_shims_noreplan/` also disables the re-planner, for the
   test without re-planning.
3. **Analyse.** `analyse_missions.m` turns the logs into the metrics, the sweep
   tables and the report figures. Ground truth is the vehicle pose before the
   simulator adds sensor noise.

## Scripts

| Script | Purpose |
|---|---|
| `batch_missions.m`, `run_batch.sh` | Run seeded missions and save the logs |
| `analyse_missions.m` | Metrics, sweep tables, controller statistics and Figures 2, 4, 6 and 7 |
| `noise_characterise.m` | Statistics of the simulator's pose and speed noise |
| `kf_tune.m` | Replays the Kalman filter offline over 60 noise settings |
| `quick_eval.m` | Quick comparison of mission sets against the noise-free pose |
| `ablation.m` | Missions 1 to 8 with the re-planner switched off |
| `extra_checks.m`, `A_star_count.m` | A* against Dijkstra (points expanded) |
| `order_check.m` | Straight-line against driving-distance waypoint ordering |
| `reactive_gain_variants/` | Avoidance code with the reactive gains halved and doubled |

## Results (`results/`)

| File | Report section |
|---|---|
| `missions_25/metrics.csv`, `summary.txt` | Table III and Results (25 missions) |
| `missions_25/ctrl_stats.txt` | Controller: measured behaviour |
| `estimator/noise.txt`, `kf_tune.txt` | State estimator: noise and filter tuning |
| `sweeps/speed_and_capture_radius.csv` | Table IV |
| `sweeps/controller_gains.csv` | Controller: gain tuning |
| `sweeps/reactive_gains.csv` | Collision avoidance: reactive gains |
| `sweeps/no_replanning.txt` | Results: avoidance layers |
| `astar_vs_dijkstra.txt` | Path planning: reasons for choosing the algorithm |
| `waypoint_ordering.txt` | Path planning: waypoint ordering |

The raw mission logs (`.mat`, about 1 MB each) are not included; rerunning
`run_batch.sh` with the same seeds reproduces them.
