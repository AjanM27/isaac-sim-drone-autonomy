# Contributing

Contributions to simulation portability, flight dynamics, ROS integration,
testing, mapping and exploration are welcome.

## Development setup

Follow [docs/INSTALLATION.md](docs/INSTALLATION.md), then create a branch:

```bash
git switch -c feature/short-description
python3 -m venv .venv/dev
.venv/dev/bin/pip install -r requirements-dev.txt
```

## Before opening a pull request

```bash
make check
```

For changes affecting flight or mapping, also provide the relevant repeatable
benchmark result. Do not claim improved stability from a single visual run.
Report the baseline and candidate parameters, test protocol, RTF, failures and
generated metrics. Do not commit `results/`, downloaded assets or Docker
images.

## Design rules

- ArduPilot remains the flight controller; operator/RL/exploration clients send
  high-level position or velocity commands rather than bypassing it with motor
  forces.
- Ground truth may evaluate an estimator but must not silently feed SLAM or
  navigation.
- Preserve offline repeat launches: new remote assets require a checksummed
  manifest entry and installer support.
- Keep simulator, ROS and flight-stack timing explicit and measured.
- Add a unit or integration regression test for every bug fix where practical.
- Never weaken ArduPilot pre-arm checks to hide a sensor/EKF integration bug.

## Pull requests

Describe:

1. the problem and intended result;
2. changed architecture or interfaces;
3. exact validation commands;
4. before/after metrics for dynamic changes;
5. hardware, driver and Isaac Sim version used.

Small focused commits are preferred. Project code uses the Apache-2.0 license;
third-party files retain their original license and attribution.
