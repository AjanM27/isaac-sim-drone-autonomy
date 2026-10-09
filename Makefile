.PHONY: setup doctor test lint check assets launch teleop stop hospital explore
PYTHON ?= python3

setup:
	./scripts/setup_ubuntu.sh --accept-eula

doctor:
	./scripts/doctor.sh

test:
	$(PYTHON) -m unittest discover -s docker/flight_control -p 'test_*.py'

lint:
	bash -n $$(find scripts docker simulation/room_quadcopter -type f -name '*.sh' -print)
	$(PYTHON) -m compileall -q scripts docker/flight_control docker/slam/isaac_slam simulation/room_quadcopter/scripts
	git diff --check

check: lint test

assets:
	$(PYTHON) scripts/download_assets.py --profile all --verify-only

launch:
	./simulation/room_quadcopter/run_room_ardupilot.sh

teleop:
	bash docker/flight_control/teleop.sh

stop:
	bash docker/flight_control/stop.sh

hospital:
	./simulation/room_quadcopter/run_hospital_demo.sh

explore:
	./simulation/room_quadcopter/run_hospital_exploration.sh
