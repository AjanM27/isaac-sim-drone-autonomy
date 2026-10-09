# Fast PID tuning

For quick exploration, PID changes can be applied to a running ArduPilot
instance. MAVROS now performs an explicit parameter pull first, so a fresh
container cannot fail with an incomplete/undeclared parameter cache.

```bash
# Show available profiles and the live controller values
bash docker/flight_control/pid_tune.sh profiles
bash docker/flight_control/pid_tune.sh show

# Apply a complete profile, or change one value
bash docker/flight_control/pid_tune.sh apply baseline
bash docker/flight_control/pid_tune.sh set PSC_ACCZ_D 0.005

# Run the same hover and four-direction trial every time
bash docker/flight_control/run_pid_trial.sh baseline_01
bash docker/flight_control/compare_pid_trials.sh
```

Use a clean simulator, EKF, and SITL process for any result used to select a
winner. This command owns the full lifecycle and preserves the Isaac log beside
the CSV/JSON result:

```bash
bash docker/flight_control/run_tuning_trial_fresh.sh \
  checkpoint1_final final_repeat_01 \
  --hover-seconds 25 --warmup-seconds 12 --speed 0.7
```

The benchmark uses a captured position setpoint during hover by default. Pass
`--hover-policy velocity` only when deliberately comparing the older
zero-velocity idle behavior.

Each trial lands the drone and writes both a per-sample CSV and a metrics JSON
under `simulation/room_quadcopter/results/pid_tuning/`. The CSV includes phase,
Isaac world XYZ, roll/pitch/yaw, MAVROS XYZ, and MAVROS velocity. Do not run
keyboard teleoperation at the same time as an automated PID trial.

Once a profile has won repeated trials, copy its values into
`isaac_sim_tuned.parm`; live MAVROS changes are otherwise only guaranteed for
the current SITL instance.

## Automatic four-decimal search

`run_pid_optimizer.sh` performs a bounded coordinate search instead of manual
gain guessing. Every candidate is rounded to four decimal places, launched in
a fresh Isaac/SITL session, evaluated by a machine-readable objective, and
compared using repeated trials. Search state and every verdict are saved under
`simulation/room_quadcopter/results/pid_tuning/autotune/<run-id>/`, so an
interrupted search can resume exactly where it stopped.

Start with one controller group. Do not run teleoperation or another simulator
at the same time:

```bash
bash docker/flight_control/run_pid_optimizer.sh \
  --run-id checkpoint1_xy_precision \
  --groups horizontal_velocity \
  --repeats 2 --max-evaluations 12 --max-runtime 90m

# Resume the same bounded search
bash docker/flight_control/run_pid_optimizer.sh \
  --run-id checkpoint1_xy_precision --resume \
  --groups horizontal_velocity \
  --repeats 2 --max-evaluations 20 --max-runtime 60m
```

The objective combines hover XY/Z error, yaw oscillation, braking distance,
post-stop climb, settling time, and repeat-to-repeat variation. A candidate
must also pass hard limits for all safety-critical metrics. The incumbent is
updated only when the score improves by at least `0.015`. Parameters are not
persisted unless `--promote` is explicitly supplied, and promotion refuses
single-run results:

```bash
bash docker/flight_control/run_pid_optimizer.sh \
  --run-id checkpoint1_xy_precision --resume \
  --groups horizontal_velocity --repeats 2 \
  --max-evaluations 20 --max-runtime 60m --promote
```

Search bounds and initial steps are versioned in `pid_search_space.json`; the
baseline is read directly from `isaac_sim_tuned.parm` at the start of a new
run, so it cannot silently diverge from the controller SITL actually loads.
Search horizontal velocity, horizontal position, vertical, and yaw as separate
runs; changing all loops simultaneously makes a result difficult to attribute
and unsafe to promote. A resumed run also rejects changed repeats, groups,
timings, or speed because those would make candidate scores incomparable.

The validated default is the `checkpoint1_final` profile. Rejected profiles
remain available only when they document a useful comparison; unsafe
high-authority experiments are not kept as selectable profiles.

## Command interface

Teleoperation and automated trials use `mavros_msgs/PositionTarget` on
`/drone_1/setpoint_raw/local`:

- While moving, position fields are ignored and XYZ velocity is commanded. The
  teleoperator rotates forward/left camera-frame input into the local ENU frame
  from the aircraft's live yaw and the Iris camera's 90-degree mount offset.
- While hovering or braking, velocity fields are ignored and fixed XYZ position
  is commanded.
- A stop first commands zero velocity until motion is low (or the bounded brake
  timeout expires), then captures and holds the resulting position. This avoids
  circular correction toward a position captured while momentum was still high.
- Both modes command `pi/2` NED yaw, which corresponds to zero yaw in Isaac's
  ENU world. This explicit conversion prevents the former 90-degree heading
  change and keeps heading active during translation.

ArduPilot remains responsible for position/velocity, attitude, angular-rate,
motor mixing, and PWM control. The keyboard and future RL policy do not command
motors directly.

## Validated precision profile (2026-09-08)

The final tuning uses `PSC_POSXY_P=1.0500`, `PSC_VELXY_P=2.2000`,
`PSC_VELXY_I=1.0000`, `PSC_VELXY_D=0.2500`, `PSC_JERK_XY=20.0000`, and
`WPNAV_ACCEL=500.0000`. The altitude and yaw loops remain
`PSC_POSZ_P=0.9000`, `PSC_VELZ_P=4.0000`, `PSC_ACCZ_P/I/D=0.4000/0.8000/0`,
and `ATC_RAT_YAW_P/I=0.0400/0.0050`.

The decisive fix was timing, not a large gain increase. The JSON bridge now
drops missed send deadlines instead of replaying stale PhysX samples in a
burst. SITL starts with a clean EEPROM and uses deterministic 20 Hz, zero-lag
GPS plus tighter EKF measurement noise. This removed intermittent half-metre
hover excursions that PID changes could not safely correct.

| Metric | Final measured result |
|---|---:|
| 61.7 s hover, XY maximum radius | 0.0500 m |
| 61.7 s hover, XY RMS | 0.0233 m |
| 61.7 s hover, Z peak-to-peak | 0.0148 m |
| Two 30 s repeats, worst XY radius | 0.0603 m |
| Two 30 s repeats, mean XY RMS | 0.0270 m |
| Two repeats, mean estimator-vs-truth XY RMS | 0.0173 m |
| Two repeats, mean yaw peak-to-peak | 1.8492 deg |
| 0.7 m/s braking, worst travel | 0.3320 m |

The `<0.10 m` criterion applies to position hover. Braking distance is reported
separately: at 0.7 m/s the aircraft must tilt and dissipate momentum, so an
instantaneous 0.10 m stop is not a physically valid acceptance requirement.
The two final run summaries and evaluator output are under
`simulation/room_quadcopter/results/pid_tuning/final_validation/`.
