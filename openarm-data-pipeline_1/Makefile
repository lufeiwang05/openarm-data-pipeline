.PHONY: install can monitor demo export serve test
install:
	pip install -e .
can:           ## task 1: bring up CAN-FD + zero position (mock)
	python -m openarm_pipeline.can.cli can_configure
monitor:       ## task 2: live joint telemetry sample (mock)
	python -m openarm_pipeline.can.cli monitor -i can0
demo:          ## tasks 2-4: record a synthetic episode to MCAP
	python scripts/demo_record.py
export:        ## record then export to OpenArm native dataset format
	python scripts/demo_record.py --export
serve:         ## tasks 4-5: REST API + dashboard at http://localhost:8000
	uvicorn openarm_pipeline.api.server:app --reload
test:
	pytest -q
